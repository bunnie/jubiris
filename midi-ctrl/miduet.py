#!/usr/bin/env python3
# Midi control:
#  - sudo apt-get install python3-pip
#  - sudo apt-get install alsa-tools alsa-utils libasound2-dev
#  - python3 -m pip install mido
#  - python3 -m pip install python-rtmidi  # note, NOT the 'rtmidi' package, it is incompatible
#  - sudo ln -s /usr/lib/x86_64-linux-gnu/alsa-lib/ /usr/lib64/alsa-lib  # fixes a library path issue that might just be a problem with ubuntu 22.04
# Duet3D serial:
#  - python3 -m pip install pyserial
# Image processing:
#  PyQt5 is not natively compatible with OpenCv. To work around this, the following procedure is necessary.
#  - This is tested in a Python 3.10 enviornment -- the pins below may not work with older versions of Python
#  - Clean out any old opencv or pyqt5 installation:
#    - `python3 -m pip list | grep -i cv`, `python3 -m pip list | grep -i qt`; `python3 -m pip uninstall` everything that was listed.
#    - python3 -m pip install opencv-python==4.8.1.78
#    - python3 -m pip install pyqt5==5.15.0
#    - adjust the `envpath` variable below to point to the `cv2` envpath in your environment. The particular adjustments
#      you'll have to make are at least for your home directory and also the version of Python you're using.
#  - python3 -m pip install numpy qimage2ndarray

# The headless version of opencv-python is needed because it bundles a Qt plugin that is the wrong version
#
# Helpful: https://docs.google.com/document/d/1zeRPklp_Mo_XzJZUKu2i-p1VgfBoUWF0JKZ5CzX8aB0/edit#heading=h.pb5rw66aumst
# ^^^ "Unofficial Communications Protocol for Akai MidiMix controller"

import argparse
import serial
import serial.tools.list_ports
import mido
import json
import time
import pprint
import logging
import re

from cam import cam
from threading import Thread, Event
import queue
import numpy as np
import math
import pandas as pd

import datetime

from light import Light, MAX_LIGHT
from piezo import Piezo, PIEZO_MAX_CODE, PIEZO_MAX_VOLT, PIEZO_UM_PER_LSB, PIEZO_UM_PER_VOLT, MAX_PIEZO
from jubilee import Jubilee, Poi, MAX_POI, MIN_ROTATION_ANGLE, MAX_ROTATION_ANGLE, \
    CONTROL_MIN_ROTATION_ANGLE, CONTROL_MAX_ROTATION_ANGLE, MIN_JUBILEE_STEP_MM
from cam import FOCUS_AREA_PX
from midi import Midi
from PyQt5.QtCore import QRect

import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

# Device-specific mapping of serial numbers of FTDI RS232 adapters
# Ensures that the correct port is picked even if the device node changes randomly.
SERIAL_NO_LED   = 'FTCYSOO2'
SERIAL_NO_PIEZO = 'FTCYSQ4J'
SERIAL_NO_MOT0  = 'FTD3MKXS'
SERIAL_NO_MOT1  = 'FTD3MLD8'

SCHEMA_STORAGE = 'miduet-config.json'
SCHEMA_VERSION = 3
MAX_GAMMA = 2.0

# Tuning method:
# 1. Set machine to any region of interest.
# 2. Use piezo knob to try to find best focus exclusively looking at metric graph
# 3. Adjust laplacian & filter to get the strogest positive signal for "focusedness".
#    Note that some settings actually create a strong signal for out of focusedness, and
#    the precise setting will depend a bit on the fabrication process being imaged.
#    Other notes: a standard cell region will do worse with over-filtering (>7), but
#    coarser regions like pad drivers might do better. Try and pick a compromise.
# 4. Observe the steady state variance, and set the FOCUS_VARIANCE_THRESH to that.
# 5. Go to the top left, and focus the region. Set is as the first POI.
# 6. Hit the "mid-z" button to put the mechanical Z into the mid zone of the piezo actuator
# 7. Store the first POI again with mid-z set.
# 8. Go to the bottom left. Focus, and store as POI 2
# 9. Go to the bottom right. Focus, and store as POI 3.
# 10. Go back to POI 1, which is the start of stitch.
# 11. Make sure we're focused.
# 12. Start the stitching run.

FOCUS_VARIANCE_THRESH = 11.0 # 1-sigma acceptable deviation for focus data. This value strongly depends on the laplacian & filtering.
FOCUS_MAX_HISTORY = 2000
FOCUS_SLOPE_SEARCH_STEPS = 1 # causes it to re-analyze every step
FOCUS_STEP_UM = 5.0 # piezo step in microns during focus searching
FOCUS_MIN_SAMPLES = 5
FOCUS_PIEZO_SETTLING_S = 0.2 # empirically determined settling time
FOCUS_MAX_RETRIES = 15
FOCUS_MAX_EXCURSION_MM = 0.2
FOCUS_STEPS_MARGIN = 2 # minimum number of steps required to define one side of the focus curve
AUTOFOCUS_SAFETY_MARGIN_MM = 0.1 # max bounds on autofocus deviation from scanned range
FOCUS_SETTLING_WINDOW_S = 1.0 # maximum window of data to consider if the machine has settled
FOCUS_SETTLING_MIN_SAMPLES = 5 # no meaningful settling metric with fewer than this number of samples
FOCUS_INCREMENTAL_RANGE_UM = 10
FOCUS_ABSOLUTE_RANGE_UM = 25 # total deviation allowable from the projected Z-plane
FOCUS_RSQUARE_LIMIT = 0.03 # multiply by mean of focus metric to get max error on fit (think of as %age of metric for fit)
FOCUS_RATIO_LIMIT = 0.995 # minimum ratio of actual vs predicted focus metric

cam_quit = Event()

class Gamma:
    def __init__(self):
        self.gamma = 1.0

# A class for shared state to name image frames. Protected by
# the auto_snap_event and auto_snap_done Events. The object is
# safe to change by the controller until an auto_snap_event is
# triggered; after which, it must wait until auto_snap_done triggers
# to update state.
#
# Contrary to its name, its function is not just to name images,
# but it has also absorbed functions to channel data from the control
# thread to the UI thread, such as Quit state and focus area.
class ImageNamer:
    def __init__(self):
        self.name = None # base name
        self.x = None  # jubilee x/y/z in mm
        self.y = None
        self.z = None
        self.p = None  # raw piezo step value
        self.i = None  # intensity of light
        self.t = None  # theta of light
        self.j = None  # remote intensity of light
        self.u = None  # remote theta of light
        self.a = None  # rotation of stage
        self.rep = None # repetitions of the same spot
        self.cur_rep = None
        self.quit = False # flag to indicate if we're exiting
        self.dummy = False
        self.focus_area = (0, 0) # (x, y) tuple encoding focus area snapping
        self.w = 3840 # expected image total width
        self.h = 2160 # expected image total height
        self.f = 0 # polynomial fit
        self.s = 0 # focus score metric
        self.v = 0 # predicted vs actual ratio
    def is_init(self):
        return self.name is not None and self.x is not None and self.y is not None \
        and self.z is not None and self.p is not None \
        and self.i is not None and self.t is not None
    # Focus area is an (x, y) tuple where each specifies a sign of an offset for
    # the focus area as follows:
    FOCUS_RIGHT = 1
    FOCUS_CENTER = 0
    FOCUS_LEFT = -1
    FOCUS_TOP = -1
    FOCUS_BOTTOM = 1
    # For example, (0, 0) means center; (1, 0) means "align to right center",
    # (1, -1) means "align to top right". Only the sign of the argument matters,
    # the magnitude is ignored.
    def set_focus_area(self, fa: (int, int)):
        self.focus_area = fa
    def get_focus_rect(self):
        if self.focus_area[0] == ImageNamer.FOCUS_CENTER: # center
            left = self.w // 2 - FOCUS_AREA_PX // 2
        elif self.focus_area[0] < 0: # align left
            left = 0
        else: # align right
            left = self.w - FOCUS_AREA_PX
        if self.focus_area[1] == ImageNamer.FOCUS_CENTER:
            top = self.h // 2 - FOCUS_AREA_PX // 2
        elif self.focus_area[1] < 0: # top
            top = 0
        else: # bottom
            top = self.h - FOCUS_AREA_PX
        return QRect(left, top, FOCUS_AREA_PX, FOCUS_AREA_PX)
    def get_name(self):
        if self.cur_rep is not None:
            r = str(self.cur_rep)
        else:
            r = '1'
        return f'x{self.x:0.2f}_y{self.y:0.2f}_z{self.z:0.2f}_p{int(self.p)}_i{int(self.i)}_t{int(self.t)}_j{int(self.j)}_u{int(self.u)}_a{self.a:0.1f}_r{int(r)}_f{self.f:0.1f}_s{int(self.s)}_v{self.v:0.3f}'

# evaluate if the collected focus data meets our quality requirements (is the machine settled)
# returns True if the data is usable, as well as the statistics on the data
# returns False if more data should be collected. Bad data is marked as such.
#
# focus data is max-weighted: focus problems only reduce the focus metric. It is highly
# improbable that any noise or focus problem accidentally creates a positive noise spike
# in the metric.
def is_focus_good(step_data):
    use_max_metric = True
    metric = step_data['focus'].max()
    logging.debug(f"{step_data[['focus', 'quality']]}")
    logging.debug(f"max: {metric}")
    step_data.loc[:, 'quality'] = 'good'
    out_of_range_indices = \
            (step_data['focus'] < metric - FOCUS_VARIANCE_THRESH * 5)
    num_bad = len(step_data[out_of_range_indices])

    if num_bad / len(step_data) > 0.8 and len(step_data) > 10:
        # heuristic: if we have a lot of "bad" samples but a lot of data that's pretty tightly grouped,
        # suspect that the "max" is an outlier. Switch to a "mean" metric.
        metric = step_data['focus'].mean()
        out_of_range_indices = \
                (step_data['focus'] < metric - FOCUS_VARIANCE_THRESH * 2) \
                | (step_data['focus'] > metric + FOCUS_VARIANCE_THRESH * 2)
        num_bad = len(step_data[out_of_range_indices])
        use_max_metric = False

    step_data.loc[out_of_range_indices, 'quality'] = 'bad'
    if len(step_data) - num_bad < FOCUS_MIN_SAMPLES:
        logging.warning(f"High variance on focus data: {num_bad} of {len(step_data)} are out of range")
        logging.debug(f"{step_data[['focus', 'quality']]}")
        return False, None, None
    else:
        good_data = step_data[step_data['quality'] == 'good']
        if use_max_metric:
            metric = good_data['focus'].max()
        else:
            metric = good_data['focus'].mean()
        logging.debug(f"{good_data}")
        return True, metric, good_data['focus'].std()

# target for a thread that just consumes the cam_quit event and processes the safe exit sequence
def quitter(jubilee, light, piezo, cam_quit):
    cam_quit.wait()
    jubilee.motors_off()
    light.send_cmd("I 0")
    piezo.set_code(0)
    logging.info("Safe quit reached!")

def total_z_mm_from_parts(jub_z, piezo_code):
    return jub_z - (piezo_code * PIEZO_UM_PER_LSB) / 1000.0

# ParamTracker tracks a parameter over successive calls to `update()`.
# When the parameter stops changing, the `action` function is invoked.
class ParamTracker():
    def __init__(self, action):
        self.action = action
        self.param = None # uninit state
        self.last_param = None
        self.is_changing = False

    def update(self, val):
        self.param = val
        self.is_changing = True

    def poll(self):
        if self.is_changing:
            if self.param == self.last_param:
                self.action(self.param)
                self.is_changing = False
        self.last_param = self.param

# primary control loop for IRIS
class Iris():
    def __init__(self,
        # all args except for args is shared state with other threads
        args, jubilee, midi, light, piezo, gamma, image_name,
        auto_snap_done, auto_snap_event, schema,
        focus_queue, jubilee_state, fine_focus_event
    ):
        if args.mag == 5:
            stepsize = 0.5
        elif args.mag == 10:
            stepsize = 0.3
        elif args.mag == 20:
            stepsize = 0.1
        else:
            logging.error("Magnification must be one of [5, 10, 20]")
            exit(0)

        self.args = args
        self.stepsize = stepsize
        self.jubilee = jubilee
        self.midi = midi
        self.light = light
        self.piezo = piezo
        self.gamma = gamma
        self.image_name = image_name
        self.auto_snap_done = auto_snap_done
        self.auto_snap_event = auto_snap_event
        self.schema = schema
        self.focus_queue = focus_queue
        self.jubilee_state = jubilee_state
        self.fine_focus_event = fine_focus_event
        self.focus_automation = False

        # only setup the focus_df as the shared value between the main loop and the focus routine
        # the rest of the focus variables should be setup in the focus "IDLE" state
        self.focus_df = pd.DataFrame({
            "time" : [],  # datetime at time of measurement
            "focus" : [], # raw value of variance of laplacian of focus area
            "state" : [], # the state of the focus machine at the time of the measuremen
            "piezo" : [], # raw piezo code
            "z" : [],     # z-motor height
            "quality" : [], # quality check metric: unchecked, good, bad
            "expo_g" : [], # exposure gain
            "expo_t" : [], # exposure time
        })
        self.focus_state = "IDLE"
        self.polyfit = 0
        self.final_score = 0
        self.predicted_metric = 0
        self.focus_ratio = 0

    def all_leds_off(self):
        # turn off all controller LEDs
        for control_node in self.schema.values():
            if type(control_node) is str:
                if 'note=' in control_node:
                    note_id = int(control_node.split('=')[1])
                    self.midi.set_led_state(note_id, False)

    def current_pos_as_poi(self):
        return Poi(self.jubilee.x, self.jubilee.y, self.jubilee.z, self.piezo.code)

    def total_z_mm(self):
        return self.jubilee.z - (self.piezo.code * PIEZO_UM_PER_LSB) / 1000.0

    # Split a total Z value into a jubilee and piezo setting
    def partition_z_mm(self, z_proposed_mm):
        if self.jubilee.z - z_proposed_mm > 0 and self.jubilee.z - z_proposed_mm < (PIEZO_MAX_CODE * PIEZO_UM_PER_LSB / 1000):
            # see if we can get to zp without adjusting z at all
            z = self.jubilee.z
            p_lsb = ((z - z_proposed_mm) * 1000) / PIEZO_UM_PER_LSB
            logging.debug(f"p_lsb only: {z}, {p_lsb}")
        else:
            Z_INCREMENT = 1/0.02
            z = math.ceil(z_proposed_mm * Z_INCREMENT) / Z_INCREMENT
            # make up the rest with the piezo
            p_lsb = ((z - z_proposed_mm) * 1000) / PIEZO_UM_PER_LSB
            logging.debug(f"z and p_lsb: {z}, {p_lsb}")
        return (z, p_lsb)

    # Nudges the z distance, preferring to move the piezo before invoking jubilee
    # nudge is in units of microns
    # Returns False if we did not have to adjust machine Z
    # True if there was a change
    def smart_nudge_z_um(self, nudge_um):
        next_code = int(-nudge_um / PIEZO_UM_PER_LSB + self.piezo.code)

        # step if possible; if not adjust Z-axis. this might be dubious due to z-axis inaccuracy
        # (may be better to just determine the overall range prior to focus sweep and try to center things up)
        if self.piezo.is_code_valid(next_code):
            self.piezo.set_code(next_code)
            return False
        else:
            logging.info(f"Adjusting machine z: {next_code} is out of range @ {self.jubilee.z}mm")
            if next_code > PIEZO_MAX_CODE / 2: # going too high
                # z-axis has mapping of lower values are closer to objective, so subtract a min step
                self.jubilee.step_axis('z', -MIN_JUBILEE_STEP_MM)
                # recalculate the next code
                next_code = self.piezo.code - (MIN_JUBILEE_STEP_MM * 1000.0) / PIEZO_UM_PER_LSB
                self.piezo.set_code(next_code)
                logging.info(f"Code is now {next_code} @ {self.jubilee.z}mm")
            else: # going too low
                self.jubilee.step_axis('z', MIN_JUBILEE_STEP_MM)
                next_code = self.piezo.code + (MIN_JUBILEE_STEP_MM * 1000.0) / PIEZO_UM_PER_LSB
                self.piezo.set_code(next_code)
                logging.info(f"Code is now {next_code} @ {self.jubilee.z}mm")
            return True

    def smart_set_z_mm(self, total_z_mm):
        (jubilee_z, piezo_code) = self.partition_z_mm(total_z_mm)
        logging.info(f"smart_set_z: {total_z_mm} -> ({jubilee_z:0.2f}, {piezo_code})")
        self.jubilee.set_axis('z', jubilee_z)
        self.piezo.set_code(piezo_code)

    def set_mid_z(self):
        pois = []
        for p in self.jubilee.poi:
            if p is not None:
                pois += [p]
        if len(pois) < 1:
            logging.info("No POI set, can't center Z")
            return
        total_zs = []
        for p in pois:
            total_zs += [p.z - (p.piezo * PIEZO_UM_PER_LSB) / 1000.0]
        avg_z = np.mean(total_zs)
        target_piezo = MAX_PIEZO / 2
        machine_mid_z = avg_z + (target_piezo * PIEZO_UM_PER_LSB) / 1000.0
        z_increment = 1/0.01
        machine_mid_z = math.ceil(machine_mid_z * z_increment) / z_increment
        piezo_mid_z = ((machine_mid_z - avg_z ) * 1000) / PIEZO_UM_PER_LSB
        self.jubilee.set_axis('z', machine_mid_z)
        self.piezo.set_code(piezo_mid_z)
        logging.info(f"Z list: {total_zs}")
        logging.info(f"Z avg: {avg_z}, Z jubilee: {machine_mid_z:0.02f}, Piezo code: {piezo_mid_z}")

    # Requires that self.settling_start has been set!
    def is_settled(self):
        settling_data = self.focus_df[(self.focus_df['time'] >= (self.settling_start))]
        if len(settling_data) < FOCUS_SETTLING_MIN_SAMPLES:
            return False
        else:
            var = settling_data['focus'].std()
            logging.debug(f"Setting metric: {var} @ {len(settling_data)} samples")
            # final score is set here because is_settled() is the last thing called before an image is taken
            self.final_score = settling_data['focus'].mean()
            return var < 2 * FOCUS_VARIANCE_THRESH

    def wait_for_machine_settling(self):
        # wait for the machine to settle
        expo_time_s = self.focus_df.loc[self.focus_df['time'].idxmax()]['expo_t'] / 1000
        # make sure the settling window is at least long enough to capture the minimum number of frames!
        if expo_time_s * (FOCUS_SETTLING_MIN_SAMPLES + 1) > FOCUS_SETTLING_WINDOW_S:
            settling_window = expo_time_s * (FOCUS_SETTLING_MIN_SAMPLES + 1)
        else:
            settling_window = FOCUS_SETTLING_WINDOW_S
        self.settling_start = datetime.datetime.now()
        while True:
            time.sleep(expo_time_s)
            self.fetch_focus_events()
            if self.is_settled():
                break
            now = datetime.datetime.now()
            if (now - self.settling_start).total_seconds() > settling_window:
                # update so that we have a sliding window for settling
                self.settling_start = now - datetime.timedelta(seconds=settling_window)

    def set_image_name(self, rep=1):
        self.image_name.x = self.jubilee.x
        self.image_name.y = self.jubilee.y
        self.image_name.z = self.jubilee.z
        self.image_name.p = self.piezo.code
        self.image_name.i = self.light.intensity_local
        self.image_name.t = self.light.angle_local
        self.image_name.j = self.light.intensity_remote
        self.image_name.u = self.light.angle_remote
        self.image_name.a = self.jubilee.r
        self.image_name.cur_rep = None
        self.image_name.rep = rep
        self.image_name.f = self.polyfit
        self.image_name.s = self.final_score
        self.image_name.v = self.focus_ratio

    def automate_flat_image(self):
        # check that the POIs have been set
        if self.jubilee.poi[0] is None or self.jubilee.poi[1] is None:
            logging.warning("POIs have not been set, can't automate!")
            return
        if self.jubilee.poi[2] is None:
            # simplify edge cases by just duplicating to create our third POI
            self.jubilee.poi[2] = self.jubilee.poi[1]
        # plan the path of steps
        min_x = min([i.x for i in self.jubilee.poi])
        max_x = max([i.x for i in self.jubilee.poi])
        min_y = min([i.y for i in self.jubilee.poi])
        max_y = max([i.y for i in self.jubilee.poi])

        x_path = np.arange(min_x, max_x, self.stepsize)
        if len(x_path) == 0: # this happens if min == max
            x_path = [min_x]
        # make sure we include the end interval in the photo region
        if max(x_path) < max_x:
            x_path = np.append(x_path, max(x_path) + self.stepsize)
        # These fail due to floating point rounding errors :P
        # assert(max(x_path) >= max_x)
        y_path = np.arange(min_y, max_y, self.stepsize)
        if len(y_path) == 0: # handle min == max
            y_path = [min_y]
        # make sure we include the end interval in the photo region
        if max(y_path) < max_y:
            y_path = np.append(y_path, max(y_path) + self.stepsize)
        # assert(max(y_path) >= max_y)

        # derive Z-plane equation
        p = []
        for i in range(3):
            p += [np.array((self.jubilee.poi[i].x, self.jubilee.poi[i].y,
                            total_z_mm_from_parts(self.jubilee.poi[i].z, self.jubilee.poi[i].piezo)))]

        if np.array_equal(p[1], p[2]):
            p[2][0] += 0.001 # perturb the second point slightly to make the equations solvable in case only two POI set
            p[2][1] += 0.001 # perturb the second point slightly to make the equations solvable in case only two POI set

        v1 = p[1] - p[0]
        v2 = p[2] - p[0]
        normal_vector = np.cross(v1, v2)
        a, b, c = normal_vector
        d = -(a * p[0][0] + b * p[0][1] + c * p[0][2])

        logging.info(f"stepping x {x_path}")
        logging.info(f"stepping y {y_path}")

        for i, x in enumerate(x_path):
            # track y in a serpentine to reduce machine movement (otherwise y-jogs from max to min every strip)
            if i % 2 == 0:
                y_serpentine = y_path
            else:
                y_serpentine = y_path[::-1]
            for y in y_serpentine:
                # pick the focus area based on the trajectory: focus towards the center of the chip
                # when on periphery, because the region off the margin of the chip is not suitable for focus
                if x == x_path.min():
                    focus_area_x = ImageNamer.FOCUS_RIGHT
                elif x == x_path.max():
                    focus_area_x = ImageNamer.FOCUS_LEFT
                else:
                    focus_area_x = ImageNamer.FOCUS_CENTER
                if y == y_path.min():
                    focus_area_y = ImageNamer.FOCUS_BOTTOM
                elif y == y_path.max():
                    focus_area_y = ImageNamer.FOCUS_TOP
                else:
                    focus_area_y = ImageNamer.FOCUS_CENTER

                for rep in range(self.args.reps):
                    self.image_name.set_focus_area((focus_area_x, focus_area_y))

                    if not self.args.dynamic_focus:
                        # Dead-reckon to a focus point based on solving the plane equation of the points of interest.
                        # derive composite-z
                        zp = -(a * x + b * y + d) / c

                        # Compute Z partition
                        (z, p_lsb) = self.partition_z_mm(zp)

                        # check values
                        if p_lsb > PIEZO_MAX_CODE:
                            logging.warning(f"Piezo value out of bounds: {p_lsb}, aborting step")
                            continue
                        if z > 15.0 or z < 5.0: # safety band over initial value of Z=10.0
                            logging.warning(f"Z value seems hazardous, aborting step: {z}")
                            continue
                        if x > 20.0 or x < -20.0 or y > 20.0 or y < -20.0:
                            logging.warning(f"X or Y value seems hazardous, aborting: {x}, {y}")
                            continue

                        logging.info(f"Step to {x}, {y}, {zp} ({z} + {p_lsb})")
                        self.jubilee.goto((x, y, z))
                        self.piezo.set_code(p_lsb)
                    else:
                        # Re-compute autofocus at each step
                        logging.info(f"Step to {x:0.2f}, {y:0.2f}")
                        # Check if we have drifted off the ideal plane
                        zp = -(a * x + b * y + d) / c
                        if abs(self.total_z_mm() - zp) > FOCUS_ABSOLUTE_RANGE_UM / 1000.0:
                            logging.warning("Current focus seems out of range, set back to projected focus plane")
                            # Dead-reckon to a focus point based on solving the plane equation of the points of interest.
                            self.smart_set_z_mm(zp)
                        else:
                            # Otherwise, preserve the previous Z-setting
                            self.jubilee.goto((x, y, self.jubilee.z))
                        self.wait_for_machine_settling()

                        # run focus
                        before_z_machine = self.jubilee.z
                        before_z_piezo = self.piezo.code
                        before_z = self.total_z_mm()
                        focus_success = False
                        focus_iters = 0
                        while not focus_success:
                            self.focus_automation = True
                            self.fine_focus_event.set()
                            while self.fine_focus_event.is_set():
                                self.fetch_focus_events()
                                self.fine_focus()
                                time.sleep(0.01) # yield our quantum
                            self.focus_automation = False

                            # settle after each iteration, so we have a fair start to the focus algo, and we can compute focus ratio!
                            self.wait_for_machine_settling()
                            self.focus_ratio = self.final_score / self.predicted_metric

                            # if the focus step took us out of a narrow range, or if our prediction was off,
                            # guess that we really messed up and try again.
                            # note: the origin of this is that sometimes the focus will latch onto a top mark or the texture
                            # of the silicon backside if there's a lack of interesting features to look at.
                            if abs(before_z - self.total_z_mm()) > FOCUS_INCREMENTAL_RANGE_UM / 1000.0 or \
                                self.focus_ratio <= FOCUS_RATIO_LIMIT:
                                if self.focus_ratio <= FOCUS_RATIO_LIMIT:
                                    logging.warning(f"Focus ratio out of range: {self.focus_ratio:0.3f} < {FOCUS_RATIO_LIMIT}")
                                else:
                                    logging.warning(f"Focus delta out of range: {int((before_z - self.total_z_mm())*1000)} um, trying again!")
                                # reset to the actual Z/code prior, not the interpolated code from composite-Z
                                self.jubilee.set_axis('z', before_z_machine)
                                self.piezo.set_code(before_z_piezo)
                                focus_success = False
                                focus_iters += 1
                                # pick a different spot to focus every time after the first basic retry
                                if focus_iters == 2:
                                    self.image_name.set_focus_area((ImageNamer.FOCUS_CENTER, ImageNamer.FOCUS_TOP))
                                elif focus_iters == 3:
                                    self.image_name.set_focus_area((ImageNamer.FOCUS_LEFT, ImageNamer.FOCUS_TOP))
                                elif focus_iters == 4:
                                    self.image_name.set_focus_area((ImageNamer.FOCUS_LEFT, ImageNamer.FOCUS_CENTER))
                                elif focus_iters == 5:
                                    self.image_name.set_focus_area((ImageNamer.FOCUS_RIGHT, ImageNamer.FOCUS_CENTER))
                                elif focus_iters == 6:
                                    self.image_name.set_focus_area((ImageNamer.FOCUS_CENTER, ImageNamer.FOCUS_BOTTOM))
                                else:
                                    self.image_name.set_focus_area((ImageNamer.FOCUS_CENTER, ImageNamer.FOCUS_CENTER))
                                if focus_iters > 7:
                                    logging.warning("Focus failed to converge, keeping previous Z-height.")
                                    break
                            else:
                                focus_success = True

                    logging.info(f"Predicted metric: {self.predicted_metric}, actual metric: {self.final_score}, ratio: {self.focus_ratio:0.3f}")
                    # setup the image_name object
                    self.set_image_name(rep=rep+1)
                    if not self.args.dynamic_focus:
                        # wait for system to settle
                        logging.info(f"settling for {self.args.settling}s")
                        time.sleep(self.args.settling)
                    # this should trigger the picture
                    logging.info(f"triggering photos")
                    self.auto_snap_event.set()
                    self.auto_snap_done.wait()
                    logging.info(f"got photo done event")
                    # reset the flags
                    self.auto_snap_event.clear()
                    self.auto_snap_done.clear()

        # reset the head to POI 1 (the origin of the scan) - makes it easier to fix up & rescan later
        # note that nothing prevents you from swapping the POI, it just makes your life harder. "don't do that, stupid".
        self.jubilee.recall_poi(1)

    def automate_psi_image(self):
        psi_base = self.jubilee.get_rotation()
        for delta_psi in range(0, 95, 1):
            self.jubilee.set_rotation(psi_base + delta_psi)
            self.set_image_name()
            # wait for system to settle
            logging.info(f"settling for {self.args.settling}s")
            time.sleep(self.args.settling)
            # this should trigger the picture
            logging.info(f"triggering photos")
            self.auto_snap_event.set()
            self.auto_snap_done.wait()
            logging.info(f"got photo done event")
            # reset the flags
            self.auto_snap_event.clear()
            self.auto_snap_done.clear()

    def automate_theta_image(self):
        for theta in range(0, 127, 2):
            self.light.set_angle_from_control(theta, 'remote')
            self.set_image_name()
            # wait for system to settle
            logging.info(f"settling for {self.args.settling}s")
            time.sleep(self.args.settling)
            # this should trigger the picture
            logging.info(f"triggering photos")
            self.auto_snap_event.set()
            self.auto_snap_done.wait()
            logging.info(f"got photo done event")
            # reset the flags
            self.auto_snap_event.clear()
            self.auto_snap_done.clear()

    # Runs a full focus sweep. Requires that the user makes some effort to bring the piece
    # "somewhat" into focus so that there is a measurable gradient on the focus metric.
    def fine_focus(self):
        if not self.jubilee.is_on():
            logging.warning("Machine is not on, please turn on motors before attempting to focus!")
            self.fine_focus_event.clear()
            return

        self.absolute_starting_z_mm = self.total_z_mm()
        self.focus_starting_z_machine = self.jubilee.z
        self.focus_starting_z_piezo = self.piezo.code
        self.focus_starting_z_mm = self.absolute_starting_z_mm

        # do the fine focus algorithm
        if self.focus_state == "IDLE":
            logging.info("Got fine focus request")
            self.focus_state = "FIND_SLOPE"
            self.focus_steps = 0
            self.focus_retries = 0

            self.curve_df = pd.DataFrame({
                "piezo" : [],
                "z" : [],
                "focus_metric" : [],
                "focus_var" : [],
                "total_z" : []
            })
            self.step_start = datetime.datetime.now()
            self.focus_steps = 0
            self.focus_retries = 0
            self.step_direction = 1 # 1 or -1 for increase or decreasing z value; increased Z is farther from objective

            expo_time_s = self.focus_df.loc[self.focus_df['time'].idxmax()]['expo_t'] / 1000
            self.focus_sample_duration_s = FOCUS_PIEZO_SETTLING_S + FOCUS_MIN_SAMPLES * expo_time_s
        elif self.focus_state == "FIND_SLOPE":
            # safety check
            if self.total_z_mm() > self.absolute_starting_z_mm + FOCUS_MAX_EXCURSION_MM or \
            self.total_z_mm() < self.absolute_starting_z_mm - FOCUS_MAX_EXCURSION_MM:
                logging.error("Focus run aborted, Z-excursion limit reached!")
                # reset machine to original state
                self.jubilee.set_axis('z', self.focus_starting_z_machine)
                self.piezo.set_code(self.focus_starting_z_piezo)
                self.focus_state = "EXIT"
            if (datetime.datetime.now() - self.step_start).total_seconds() > self.focus_sample_duration_s * (self.focus_retries + 1):
                logging.debug(f"focus step {self.focus_steps}")

                step_data = self.focus_df[(self.focus_df['time'] >= (self.step_start + datetime.timedelta(seconds=FOCUS_PIEZO_SETTLING_S))) \
                                          & (self.focus_df['quality'] != 'bad')]
                if len(step_data) < FOCUS_MIN_SAMPLES:
                    time.sleep(0.2) # just give the machine some time to do its thing, things might be lagging...
                else:
                    is_good, metric, var = is_focus_good(step_data)
                    if is_good:
                        # store data
                        self.focus_retries = 0
                        # have to re-index to not be working on a copy of the data
                        logging.info(f"Piezo {self.piezo.code} @ z={self.total_z_mm():0.3f} | var: {var}, mean: {metric}")
                        self.curve_df.loc[len(self.curve_df)] = [
                            self.piezo.code,
                            self.jubilee.z,
                            metric,
                            var,
                            self.jubilee.z - (self.piezo.code * PIEZO_UM_PER_LSB) / 1000.0
                        ]
                        # nudge by a focus step (5um for 10x objective)
                        # positive step direction => negative z-height
                        if self.smart_nudge_z_um(FOCUS_STEP_UM * self.step_direction):
                            # We had to adjust machine Z. Restart the search entirely.
                            logging.warning("Machine Z changed. Restarting focus search.")
                            self.focus_state = "IDLE"
                            self.wait_for_machine_settling()

                        # report to UI
                        poi = self.current_pos_as_poi()
                        self.jubilee_state.put(poi, block=False)
                        # prep next step
                        self.step_start = datetime.datetime.now()
                        self.focus_steps += 1
                        if self.focus_steps >= FOCUS_SLOPE_SEARCH_STEPS:
                            self.focus_state = "ANALYZE_SLOPE"
                    else:
                        # remove the out of range data from the source self.focus_df dataframe
                        self.focus_retries += 1
                        if self.focus_retries > FOCUS_MAX_RETRIES and not self.focus_automation:
                            self.focus_df.loc[self.focus_df['time'] >= (self.step_start + datetime.timedelta(seconds=FOCUS_PIEZO_SETTLING_S)), 'quality'] = 'bad'
                            logging.error("Data quality too low for autofocus. Aborting run. Check for excessive environment vibration or camera noise.")
                            self.focus_state = "EXIT"
                        elif self.focus_retries > FOCUS_MAX_RETRIES and self.focus_automation:
                            self.step_start = datetime.datetime.now()
                            self.focus_retries = 0
                            logging.warning("Data quality problem in automation, restarting focus stream")


        elif self.focus_state == "ANALYZE_SLOPE":
            sorted_curve = self.curve_df.sort_values(by="total_z") # now sorted from lowest to highest total Z (highest to lowest piezo code)
            sorted_curve = sorted_curve.reset_index(drop=True)
            max_row = sorted_curve['focus_metric'].idxmax()
            logging.debug(f" --- max_row: {max_row}, data points: {len(self.curve_df)} ---")
            logging.debug(f"{sorted_curve}")
            if max_row >= len(self.curve_df) - FOCUS_STEPS_MARGIN:
                logging.debug("  -> increase Z")
                # best focus is toward highest total Z
                if self.step_direction != 1:
                    self.step_direction = 1
                    self.smart_set_z_mm(sorted_curve['total_z'].max() + self.step_direction * FOCUS_STEP_UM / 1000.0)
                # restart the focus run
                self.focus_starting_z_mm = self.total_z_mm()
                self.focus_steps = 0
                self.step_start = datetime.datetime.now()
                self.focus_state = "FIND_SLOPE"
            elif max_row < FOCUS_STEPS_MARGIN:
                logging.debug("  -> decrease Z")
                # best focus is toward lowest total Z
                if self.step_direction != -1:
                    self.step_direction = -1
                    self.smart_set_z_mm(sorted_curve['total_z'].min() + self.step_direction * FOCUS_STEP_UM / 1000.0)
                # restart the focus run
                self.focus_starting_z_mm = self.total_z_mm()
                self.focus_steps = 0
                self.step_start = datetime.datetime.now()
                self.focus_state = "FIND_SLOPE"
            else:
                # fit to 2nd order and find maxima; only use the points directly around the maxima
                coefficients, residuals, _rank, _sv, _rcond = np.polyfit(
                    sorted_curve.loc[max_row - FOCUS_STEPS_MARGIN:max_row + FOCUS_STEPS_MARGIN]['total_z'],
                    sorted_curve.loc[max_row - FOCUS_STEPS_MARGIN:max_row + FOCUS_STEPS_MARGIN]['focus_metric'],
                    deg=2,
                    full=True
                )
                logging.debug(self.curve_df['total_z'])
                logging.debug(self.curve_df['focus_metric'])
                logging.debug(f"coefficients: {coefficients}")
                # compute MSE of data vs polynomial
                predicted = np.polyval(coefficients, sorted_curve.loc[max_row - FOCUS_STEPS_MARGIN:max_row + FOCUS_STEPS_MARGIN]['total_z'])
                mse = mean_squared_error(sorted_curve.loc[max_row - FOCUS_STEPS_MARGIN:max_row + FOCUS_STEPS_MARGIN]['focus_metric'], predicted)
                logging.info(f"MSE fit: {mse:0.2f}")
                self.polyfit = mse
                z_maxima = -coefficients[1] / (2 * coefficients[0])
                self.predicted_metric = np.polyval(coefficients, [z_maxima])[0]
                plt.scatter(self.curve_df['total_z'], self.curve_df['focus_metric'])
                xp = np.linspace(self.curve_df['total_z'].min() - 0.01, self.curve_df['total_z'].max() + 0.01, 100)
                yp = np.polyval(coefficients, xp)
                plt.plot(xp, yp)
                # Emit debug info
                plt.savefig("focus_fit.png", dpi=300)
                plt.cla()

                if z_maxima < self.curve_df['total_z'].min() - AUTOFOCUS_SAFETY_MARGIN_MM \
                or z_maxima > self.curve_df['total_z'].max() + AUTOFOCUS_SAFETY_MARGIN_MM:
                    logging.error(f"Computed focus is bogus: {z_maxima:0.3f}, [{self.curve_df['total_z'].min():0.3f}, {self.curve_df['total_z'].max():0.3f}]")
                    # reset machine to original state before focus search
                    self.jubilee.set_axis('z', self.focus_starting_z_machine)
                    self.piezo.set_code(self.focus_starting_z_piezo)
                    self.focus_state = "IDLE" # re-run the focus algo again -- could result in infinite loop
                    self.wait_for_machine_settling()
                else:
                    if self.polyfit > FOCUS_RSQUARE_LIMIT * self.predicted_metric:
                        logging.warning(f"Residual is out of limit ({FOCUS_RSQUARE_LIMIT * self.predicted_metric:0.2f}), retrying...")
                        self.smart_set_z_mm(z_maxima) # restart at the last limit that we thought was good
                        self.focus_state = "IDLE"
                        self.wait_for_machine_settling()
                    else:
                        # servo machine to maxima
                        logging.info(f"Focus point at z={z_maxima:0.3f}mm")
                        self.smart_set_z_mm(z_maxima)
                        # check focus score
                        self.focus_state = "EXIT"

        elif self.focus_state == "EXIT":
            # Call this when we're done with fine focus algo...
            self.fine_focus_event.clear()
            self.focus_state = "IDLE"
            # Emit debug info
            self.focus_df.to_csv("focus.csv")
            self.curve_df.to_csv("curve.csv")
        else:
            logging.warning("Unrecognized focus state: f{self.focus_state}")

    def local_angle_action(self, val):
        self.light.set_angle_from_control(val, 'local')
    def remote_angle_action(self, val):
        self.light.set_angle_from_control(val, 'remote')
    def rotation_action(self, val):
        r = (float(val) / 127.0) * \
                            (CONTROL_MAX_ROTATION_ANGLE - CONTROL_MIN_ROTATION_ANGLE) + CONTROL_MIN_ROTATION_ANGLE
        self.jubilee.set_rotation(r)
    def piezo_action(self, val):
        nudge = float(val) * (MAX_PIEZO / 127.0)
        self.piezo.set_code(int(nudge))
        poi = self.current_pos_as_poi()
        self.jubilee_state.put(poi, block=False)

    def fetch_focus_events(self):
        while not self.focus_queue.empty():
            (timestamp, focus_metric, expo_gain, expo_time) = self.focus_queue.get()
            if self.jubilee.z == None:
                z_checked = 10.0
            else:
                z_checked = self.jubilee.z
            self.focus_df.loc["last"] = [timestamp, focus_metric, self.focus_state, self.piezo.code, z_checked, "unchecked", expo_gain, expo_time]
            self.focus_df = self.focus_df.reset_index(drop=True)
            if len(self.focus_df) > FOCUS_MAX_HISTORY:
                # cull some history to prevent storage problems
                self.focus_df = self.focus_df.tail(-FOCUS_MAX_HISTORY // 2)
            time.sleep(0.01) # yield quantum

    def loop(self):
        # clear any stray events in the queue
        self.midi.clear_events()
        if self.jubilee.motors_off():
            # the command timed out
            logging.warning("Machine not responsive, restarting Jubilee...")
            if self.jubilee.restart():
                logging.error("Couldn't connect to Jubilee")
                return

        refvals = {}
        curvals = {}
        for name in self.schema.keys():
            refvals[name] = None
            curvals[name] = None
        paused_axis = None
        # turn on a low illumination so the camera doesn't seem "broken"
        self.light.send_cmd(f"I {int(MAX_LIGHT / 5)}")

        self.all_leds_off()
        gamma_enabled = False
        last_gamma = 1.0

        if self.light.is_1050_set():
            self.midi.set_led_state(int(self.schema['1050 button'].replace('note=', '')), True)

        # track state for commands that should not issue until the knob stops moving
        trackers = []
        local_angle_tracker = ParamTracker(self.local_angle_action)
        trackers += [local_angle_tracker]
        remote_angle_tracker = ParamTracker(self.remote_angle_action)
        trackers += [remote_angle_tracker]
        rotation_tracker = ParamTracker(self.rotation_action)
        trackers += [rotation_tracker]
        piezo_tracker = ParamTracker(self.piezo_action)
        trackers += [piezo_tracker]

        profiling = False
        was_running_focus = False
        while True:
            if profiling:
                start = datetime.datetime.now()
            if cam_quit.is_set():
                self.all_leds_off()
                return

            # Drain the focus queue, run focus if requested
            # Note to self: variance of a static image is <5. Table vibration > 300 deviation. Focus changes ~100 deviation.
            self.fetch_focus_events()
            if self.fine_focus_event.is_set():
                self.fine_focus()
                was_running_focus = True
            else:
                if was_running_focus:
                    self.wait_for_machine_settling()
                    self.focus_ratio = self.final_score / self.predicted_metric
                    logging.info(f"Predicted metric: {self.predicted_metric}, actual metric: {self.final_score}, ratio: {self.focus_ratio:0.3f}")
                was_running_focus = False

            # Hardware control loop
            msg = self.midi.midi.poll()
            if msg is None:
                for tracker in trackers:
                    tracker.poll()

                if profiling:
                    logging.info(f"poll {datetime.datetime.now() - start}")
                time.sleep(0.01) # yield our quantum
                continue
            # hugely inefficient O(n) search on every iter, but "meh"
            # that's why we have Moore's law.
            valid = False
            for (name, control_node) in self.schema.items():
                if name == 'version':
                    if control_node != SCHEMA_VERSION:
                        print('Button mapping file has the wrong version number. Please run --set-controls to pick buttons again!')
                        exit(1)
                    else:
                        continue
                if control_node + ' ' in str(msg):
                    valid = True
                    # sliders
                    if 'control=' in control_node:
                        # extract the value
                        for i in str(msg).strip().split(' '):
                            if 'value=' in i:
                                control_value = i.split('=')[1]
                                break
                        assert(control_value is not None)
                        if name == "angle-local":
                            local_angle_tracker.update(float(control_value))
                        elif name == "brightness-local":
                            bright = float(control_value) * (MAX_LIGHT / 127.0)
                            self.light.set_intensity_local(int(bright))
                        elif name == "angle-remote":
                            remote_angle_tracker.update(float(control_value))
                        elif name == "brightness-remote":
                            bright = float(control_value) * (MAX_LIGHT / 127.0)
                            self.light.set_intensity_remote(int(bright))
                        elif name == "nudge-piezo":
                            piezo_tracker.update(float(control_value))
                        elif name == "gamma":
                            last_gamma = (float(control_value) / 127.0) * MAX_GAMMA
                            if gamma_enabled:
                                self.gamma.gamma = last_gamma
                            else:
                                self.gamma.gamma = 1.0
                        elif name == "rotation":
                            rotation_tracker.update(float(control_value))
                        else:
                            curvals[name] = control_value
                            if refvals[name] is None:
                                refvals[name] = control_value # first time through, grab the current value as the reference
                    # buttons
                    elif 'note=' in control_node:
                        note_id = int(control_node.split('=')[1])
                        if 'note_on' in str(msg):
                            # the button was hit
                            if 'zero' in name:
                                if 'x' in name:
                                    paused_axis = 'x'
                                elif 'y' in name:
                                    paused_axis = 'y'
                                elif 'z' in name:
                                    paused_axis = 'z'
                                else:
                                    logging.error(f"Internal error, invalid button name {name} on {str(msg)}")
                                    exit(1)
                                for name in refvals.keys():
                                    if paused_axis in name:
                                        refvals[name] = None
                            elif name == 'idle toggle button':
                                if self.jubilee.is_on():
                                    logging.info("Motors are OFF")
                                    self.midi.set_led_state(note_id, False)
                                    self.jubilee.motors_off()
                                else:
                                    logging.info("Motors are ON and origin is set")
                                    self.midi.set_led_state(note_id, True)
                                    self.jubilee.motors_on_set_zero()
                            elif name == 'ESTOP button':
                                for (name, val) in curvals.items():
                                    refvals[name] = val
                                print("ESTOP hit, exiting")
                                self.all_leds_off()
                                self.jubilee.estop()
                                time.sleep(5)
                                cam_quit.set()
                                return
                            elif '1050 button' in name:
                                self.midi.set_led_state(note_id, self.light.toggle_1050())
                                self.light.commit_wavelength()
                            elif 'set POI' in name:
                                if not self.jubilee.is_on():
                                    logging.warning("Please turn Jubilee on before setting a POI")
                                    continue
                                maybe_poi = re.findall(r'^set POI (\d)', name)
                                if len(maybe_poi) == 1:
                                    if int(maybe_poi[0]) == 1: # when setting POI 1, reset machine coords to 0
                                        logging.info("POI 1 set - note that this should be the top left of the scan area by convention!")
                                        self.jubilee.motors_on_set_zero()
                                    self.midi.set_led_state(note_id, True)
                                    self.jubilee.set_poi(int(maybe_poi[0]), self.piezo.code)
                                    logging.info(f"Set POI {maybe_poi[0]}. X: {self.jubilee.x:0.2f}, Y: {self.jubilee.y:0.2f}, Z: {self.jubilee.z:0.2f}, P: {self.piezo.code}, Z': {self.total_z_mm():0.3f}")
                            elif 'recall POI' in name:
                                maybe_poi = re.findall(r'^recall POI (\d)', name)
                                if len(maybe_poi) == 1:
                                    poi_index = int(maybe_poi[0])
                                    piezo_residual = self.jubilee.recall_poi(poi_index)
                                    if piezo_residual is not None:
                                        self.piezo.set_code(piezo_residual)
                                        logging.info(f"Recall POI {poi_index}. X: {self.jubilee.x:0.2f}, Y: {self.jubilee.y:0.2f}, Z: {self.jubilee.z:0.2f}, P: {self.piezo.code}, Z': {self.total_z_mm():0.3f}")
                                        poi = self.current_pos_as_poi()
                                        self.jubilee_state.put(poi, block=False)

                            elif name == 'zset button':
                                self.set_mid_z()
                            elif name == 'gamma button':
                                if gamma_enabled:
                                    self.midi.set_led_state(note_id, False)
                                    gamma_enabled = False
                                    self.gamma.gamma = 1.0
                                else:
                                    self.midi.set_led_state(note_id, True)
                                    gamma_enabled = True
                                    self.gamma.gamma = last_gamma
                            elif name == 'report position button':
                                try:
                                    logging.info(f"X: {self.jubilee.x:0.2f}, Y: {self.jubilee.y:0.2f}, Z: {self.jubilee.z:0.2f}, P: {self.piezo.code}, Z': {self.total_z_mm():0.3f}")
                                except:
                                    logging.info("Machine is IDLE or controls not synchronized")
                            elif name == 'automate button':
                                if self.args.automation == 'step':
                                    self.automate_flat_image()
                                elif self.args.automation == 'psi': # rotate the light around the vertical axis
                                    self.automate_psi_image()
                                elif self.args.automation == 'theta': # rotate the light around the vertical axis
                                    self.automate_theta_image()
                                else:
                                    logging.error("Unrecognized automation type, doing nothing.")
                                # update UI with final focus solution
                                poi = self.current_pos_as_poi()
                                self.jubilee_state.put(poi, block=False)

                                logging.info("Automation done!")
                                if self.args.auto_quit:
                                    self.jubilee.motors_off()
                                    self.all_leds_off()
                                    logging.info("Quitting controller...")
                                    cam_quit.set()
                                    return

                        elif 'note_off' in str(msg):
                            # the button was lifted
                            if 'zero' in name:
                                for name in refvals.keys():
                                    if paused_axis in name:
                                        refvals[name] = None
                                paused_axis = None

            if profiling:
                logging.info(f"parse {datetime.datetime.now() - start}")
            # now update machine position based on values
            if valid:
                for (name, val) in curvals.items():
                    if refvals[name] is not None:
                        if paused_axis is not None:
                            if paused_axis in name:
                                continue

                        delta = int(curvals[name]) - int(refvals[name])
                        refvals[name] = curvals[name] # reset ref
                        if delta > 0:
                            if 'coarse' in name:
                                step = 0.2
                            else:
                                step = 0.05
                        elif delta < 0:
                            if 'coarse' in name:
                                step = -0.2
                            else:
                                step = -0.05
                        else:
                            step = 0.0
                        if 'z' in name:
                            step = -(step / 5.0) # Z 0 is "up", make slider's direction correspond to stage direction
                        logging.debug(f"Delta of {delta} for {name}")

                        if self.jubilee.step_axis(name, step):
                            logging.warning("Motor command timeout!")
                            self.midi.clear_events()
                # update the UI thread
                poi = self.current_pos_as_poi()
                self.jubilee_state.put(poi, block=False)

def set_controls(midi_in):
    # Control schema layout
    schema = {
        'version' : SCHEMA_VERSION,
        'x-coarse' : None,
        'y-coarse' : None,
        'z-coarse' : None,
        'x-fine' : None,
        'y-fine' : None,
        'z-fine' : None,
        'gamma' : None,
        "angle-local" : None,
        "brightness-local" : None,
        "angle-remote" : None,
        "brightness-remote" : None,
        "rotation" : None,
        "nudge-piezo" : None,
        'zero-x button' : None,
        'zero-y button' : None,
        'zero-z button' : None,
        'idle toggle button': None,
        'set POI 1 button' : None,
        'set POI 2 button' : None,
        'set POI 3 button' : None,
        'recall POI 1 button' : None,
        'recall POI 2 button' : None,
        'recall POI 3 button' : None,
        '1050 button' : None,
        'quit button' : None,
        'ESTOP button' : None,
    }

    # clear any stray events in the queue
    for msg in midi_in.iter_pending():
        pass

    # now iterate through the dictionary and set buttons
    for (name, val) in schema.items():
        success = False
        if val is None:
            while not success:
                print(f"Touch control to set: {name}")
                msg = midi_in.receive()
                # print(msg)
                msg_list = str(msg).strip().split(' ')
                if 'button' in name:
                    button = True
                else:
                    button = False
                for i in msg_list:
                    if 'control=' in i:
                        if not button:
                            schema[name] = i
                            success = True
                            break
                        else:
                            print("Please select a slider or knob.")
                            break
                    elif 'note=' in i:
                        if button:
                            schema[name] = i
                            success = True
                            break
                        else:
                            print("Please select a button.")
                            break
                # drain any excess events
                print("  Control captured!")
                time.sleep(1)
                for msg in midi_in.iter_pending():
                    pass # just discard any excess events
    print("Control setting done. Schema is:")
    pprint.pprint(schema)

    try:
        with open(SCHEMA_STORAGE, "w") as config:
            config.write(json.dumps(schema))
    except:
        print(f"Error: could't write to {SCHEMA_STORAGE}")
        exit(1)

def main():
    parser = argparse.ArgumentParser(description="MIDI-to-Duet controller")
    parser.add_argument(
        "--automation", required=False, help="type of automation to run", default='step', choices=['step', 'psi', 'theta']
    )
    parser.add_argument(
        "--midi-port", required=False, help="MIDI device name to use"
    )
    parser.add_argument(
        "--set-controls", required=False, action="store_true", help="Set up MIDI controls"
    )
    parser.add_argument(
        "--loglevel", required=False, help="set logging level (INFO/DEBUG/WARNING/ERROR)", type=str, default="INFO",
    )
    parser.add_argument(
        "--no-cam", required=False, action="store_true", help="Do not start camera UI"
    )
    parser.add_argument(
        "--name", required=False, type=str, default="unnamed_chip", help="Directory name for auto-image snaps"
    )
    parser.add_argument(
        "--reps", required=False, type=int, default=1, help="Number of repetitions of shots of the current position"
    )
    parser.add_argument(
        "--mag", required=False, type=int, default=10, help="Magnification", choices=[5, 10, 20]
    )
    parser.add_argument(
        "--settling", required=False, type=float, default=5.0, help="Settling time in seconds between steps"
    )
    parser.add_argument(
        "--dynamic-focus", required=False, action="store_true", help="When set, ignore settling time and compute focus/quality metric on every image"
    )
    parser.add_argument(
        "--auto-quit", action="store_true", help="Automatically quit after automation runs"
    )
    args = parser.parse_args()

    gamma = Gamma()
    image_name = ImageNamer()
    image_name.name = args.name
    image_name.rep = args.reps
    auto_snap_event = Event()
    auto_snap_done = Event()
    fine_focus_event = Event()
    focus_score = queue.Queue(maxsize=5)
    jubilee_state = queue.Queue()
    if not args.no_cam:
        c = Thread(target=cam, args=[
            cam_quit, gamma, image_name,
            auto_snap_event, auto_snap_done,
            focus_score, args.mag, jubilee_state, fine_focus_event])
        c.start()

    numeric_level = getattr(logging, args.loglevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % args.loglevel)
    logging.basicConfig(level=numeric_level)

    possible_ports = list(serial.tools.list_ports.comports())
    led_port = None
    piezo_port = None
    mot0_port = None
    mot1_port = None
    jubilee_port = None
    for port, _desc, hwid in possible_ports:
        if SERIAL_NO_LED in hwid:
            led_port = port
        elif SERIAL_NO_PIEZO in hwid:
            piezo_port = port
        elif SERIAL_NO_MOT0 in hwid:
            mot0_port = port
        elif SERIAL_NO_MOT1 in hwid:
            mot1_port = port
        elif 'ACM' in port:
            jubilee_port = port

    logging.info(f"Found LED on {led_port}, piezo on {piezo_port}\nmot0 on {mot0_port}, mot1 on {mot1_port}, jubilee on {jubilee_port}")

    # Automatically pick a MIDI device, or take one from command line if specified
    if args.midi_port is None:
        input_list = mido.get_input_names()
        midimix_name = None
        for i in input_list:
            if 'MIDI Mix' in i:
                midimix_name = i
                break
        if midimix_name is None:
            logging.error("couldn't find Akai MIDI Mix device!")
            return
    else:
        midimix_name = args.midi_port

    # Load or create a control schema
    if args.set_controls:
        try:
            with mido.open_input(midimix_name) as midi_in:
                set_controls(midi_in)
        except Exception as e:
            logging.error("Couldn't open MIDI device, --set-controls has failed.")
            logging.error(e)
            exit(1)

        # after setting controls, exit
        logging.error("Control setting finished. Please restart without --set-controls.")
        exit(0)
    else:
        try:
            with open(SCHEMA_STORAGE, "r") as config:
                schema = json.loads(config.read())
        except:
            logging.error("Couldn't load control configuration. Try running --set-controls to setup controls.")
            exit(1)

    # Launch the main loop
    # Remember: if the Jubilee does not respond on serial, try unplugging
    # the USB cable and then power cycling the Jubilee, and then plugging
    # the USB cable back in. There is a bug in the Duet3D firmware that
    # locks up the USB port if it is plugged in while the main board comes up.
    jubilee = Jubilee(jubilee_port)
    jubilee.reset() # count on the time required to init the light controller for the Jubilee to finish resetting.
    midi = Midi(midimix_name)
    logging.info(f"Opening LED port {led_port}")
    light = Light(led_port, mot0_port, mot1_port)
    logging.info(f"Opening Piezo port {piezo_port}")
    piezo = Piezo([piezo_port])
    logging.info("MIDI-to-Jubilee Controller Starting. Motors are IDLE.")
    logging.info("---> Please press SEND ALL to initialize <---")

    # recover the Jubilee port after the reset
    possible_ports = list(serial.tools.list_ports.comports())
    for port, _desc, _hwid in possible_ports:
        if 'ACM' in port:
            jubilee_port = port
    jubilee.update_port(port)

    # wrap in 'with' so we can shut things down on exit if necessary
    with jubilee as j:
        with midi as m:
            with light as l:
                with piezo as p:
                    # build the main iris control object
                    iris = Iris(
                        args, j, m, l, p, gamma, image_name,
                        auto_snap_done, auto_snap_event, schema,
                        focus_score, jubilee_state, fine_focus_event
                    )

                    q = Thread(target=quitter, args=[jubilee, light, piezo, cam_quit])
                    q.start()
                    l = Thread(target=iris.loop, args=[])
                    l.start()
                    # when the quitter exits, everything has been brought down in an orderly fashion
                    q.join()
                    # inform the snapper thread to quit
                    image_name.quit = True
                    auto_snap_event.set()
                    logging.debug("Midi control thread reached quit")

if __name__ == "__main__":
    main()

