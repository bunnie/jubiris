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
from midi import Midi

SERIAL_NO_LED   = 'FTCYSOO2'
SERIAL_NO_PIEZO = 'FTCYSQ4J'
SERIAL_NO_MOT0  = 'FTD3MKXS'
SERIAL_NO_MOT1  = 'FTD3MLD8'

SCHEMA_STORAGE = 'miduet-config.json'
SCHEMA_VERSION = 3
MAX_GAMMA = 2.0

FOCUS_MAX_HISTORY = 2000
FOCUS_SLOPE_SEARCH_STEPS = 6
FOCUS_STEP_UM = 5.0 # piezo step in UM
FOCUS_SAMPLE_DURATION_S = 1.0
FOCUS_SAMPLE_DURATION_SAMPLES = 5
FOCUS_PIEZO_SETTLING_S = 0.2 # empirically determined settling time
FOCUS_VARIANCE_THRESH = 10.0 # 1-sigma acceptable deviation for focus data
FOCUS_MAX_RETRIES = 4
FOCUS_MAX_EXCURSION_MM = 0.2
FOCUS_STEPS_MARGIN = 2 # minimum number of steps required to define one side of the focus curve
AUTOFOCUS_SAFETY_MARGIN_MM = 0.1 # max bounds on autofocus deviation from scanned range

cam_quit = Event()

class Gamma:
    def __init__(self):
        self.gamma = 1.0

# A class for shared state to name image frames. Protected by
# the auto_snap_event and auto_snap_done Events. The object is
# safe to change by the controller until an auto_snap_event is
# triggered; after which, it must wait until auto_snap_done triggers
# to update state.
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
    def is_init(self):
        return self.name is not None and self.x is not None and self.y is not None \
        and self.z is not None and self.p is not None \
        and self.i is not None and self.t is not None
    def get_name(self):
        if self.cur_rep is not None:
            r = str(self.cur_rep)
        else:
            r = '1'
        return f'x{self.x:0.2f}_y{self.y:0.2f}_z{self.z:0.2f}_p{int(self.p)}_i{int(self.i)}_t{int(self.t)}_j{int(self.j)}_u{int(self.u)}_a{self.a:0.1f}_r{int(r)}'

def quitter(jubilee, light, piezo, cam_quit):
    cam_quit.wait()
    jubilee.motors_off()
    light.send_cmd("I 0")
    piezo.set_code(0)
    logging.info("Safe quit reached!")

def all_leds_off(schema, midi):
    # turn off all controller LEDs
    for control_node in schema.values():
        if type(control_node) is str:
            if 'note=' in control_node:
                note_id = int(control_node.split('=')[1])
                midi.set_led_state(note_id, False)

# evaluate if the collected focus data meets our quality requirements
# returns True if the data is usable
def is_focus_good(step_data):
    mean = step_data['focus'].mean()
    out_of_range = step_data[
            (step_data['focus'] < mean - FOCUS_VARIANCE_THRESH * 3) 
            | (step_data['focus'] > mean + FOCUS_VARIANCE_THRESH * 3)
        ]
    if len(out_of_range) > 0:
        logging.warning(f"Greater than 3-sigma variation detected on focus data: {len(out_of_range)} points")
    return len(out_of_range) == 0

def total_z_mm(z_mm, piezo_code):
    return z_mm - (piezo_code * PIEZO_UM_PER_LSB) / 1000.0

# Split a total Z value into a jubilee and piezo setting
def partition_z(zp, cur_jubilee_z):
    if cur_jubilee_z - zp > 0 and cur_jubilee_z - zp < (PIEZO_MAX_CODE * PIEZO_UM_PER_LSB / 1000):
        # see if we can get to zp without adjusting z at all
        z = cur_jubilee_z
        p_lsb = ((z - zp) * 1000) / PIEZO_UM_PER_LSB
        logging.info(f"p_lsb only: {z}, {p_lsb}")
    else:
        Z_INCREMENT = 1/0.02
        z = math.ceil(zp * Z_INCREMENT) / Z_INCREMENT
        # make up the rest with the piezo
        p_lsb = ((z - zp) * 1000) / PIEZO_UM_PER_LSB
        logging.info(f"z and p_lsb: {z}, {p_lsb}")
    return (z, p_lsb)

def loop(args, stepsize, jubilee, midi, light, piezo, gamma, image_name,
         auto_snap_done, auto_snap_event, schema,
         focus_queue, jubilee_state, fine_focus_event):
    # clear any stray events in the queue
    midi.clear_events()
    if jubilee.motors_off():
        # the command timed out
        logging.warning("Machine not responsive, restarting Jubilee...")
        if jubilee.restart():
            logging.error("Couldn't connect to Jubilee")
            return

    refvals = {}
    curvals = {}
    for name in schema.keys():
        refvals[name] = None
        curvals[name] = None
    paused_axis = None
    # turn on a low illumination so the camera doesn't seem "broken"
    light.send_cmd(f"I {int(MAX_LIGHT / 5)}")

    all_leds_off(schema, midi)
    gamma_enabled = False
    last_gamma = 1.0

    if light.is_1050_set():
        midi.set_led_state(int(schema['1050 button'].replace('note=', '')), True)
    
    # track state for commands that should not issue until the knob stops moving
    local_angle_changed = False
    remote_angle_changed = False
    last_local_angle_control_value = 0.0
    new_local_angle_control_value = 0.0
    last_remote_angle_control_value = 0.0
    new_remote_angle_control_value = 0.0
    rotation_changed = False
    last_rotation_value = 0.0
    new_rotation_value = 0.0
    piezo_changed = False
    last_piezo_value = 0.0
    new_piezo_value = 0.0

    focus_df = pd.DataFrame({
        "time" : [],  # datetime at time of measurement
        "focus" : [], # raw value of variance of laplacian of focus area
        "state" : [], # the state of the focus machine at the time of the measuremen
        "piezo" : [], # raw piezo code
        "z" : [],     # z-motor height
        "quality" : [], # quality check metric: unchecked, good, bad
    })
    focus_state = "IDLE"
    step_start = datetime.datetime.now()
    focus_steps = 0
    focus_retries = 0
    step_direction = 1 # 1 or -1 for increase or decreasing step

    profiling = False
    while True:
        if profiling:
            start = datetime.datetime.now()
        if cam_quit.is_set():
            all_leds_off(schema, midi)
            return
        
        # Drain the focus queue
        # Note to self: variance of a static image is <5. Table vibration > 300 deviation. Focus changes ~100 deviation.
        while not focus_queue.empty():
            (t, f) = focus_queue.get()
            if jubilee.z == None:
                z_checked = 10.0
            else:
                z_checked = jubilee.z
            focus_df.loc[len(focus_df)] = [t, f, focus_state, piezo.code, z_checked, "unchecked"]
            if len(focus_df) > FOCUS_MAX_HISTORY:
                focus_df.drop(index=0, inplace=True)
        
        if fine_focus_event.is_set():
            if not jubilee.is_on():
                logging.warning("Machine is not on, please turn on motors before attempting to focus!")
                fine_focus_event.clear()
                continue

            # do the fine focus algorithm
            if focus_state == "IDLE":
                logging.info("Got fine focus request")
                focus_state = "FIND_SLOPE"
                focus_steps = 0
                curve_df = pd.DataFrame({
                    "piezo" : [],
                    "z" : [],
                    "focus_mean" : [],
                    "focus_var" : [],
                    "total z" : []
                })
                step_start = datetime.datetime.now()
                focus_starting_z = jubilee.z
                focus_starting_piezo = piezo.code
                absolute_starting_z_mm = total_z_mm(jubilee.z, piezo.code)
            elif focus_state == "FIND_SLOPE":
                # safety check
                if total_z_mm(jubilee.z, piezo.code) > absolute_starting_z_mm + FOCUS_MAX_EXCURSION_MM or \
                total_z_mm(jubilee.z, piezo.code) < absolute_starting_z_mm - FOCUS_MAX_EXCURSION_MM:
                    logging.error("Focus run aborted, Z-excursion limit reached!")
                    focus_state = "EXIT"
                if (datetime.datetime.now() - step_start).total_seconds() > FOCUS_SAMPLE_DURATION_S:
                    logging.info(f"focus step {focus_steps}")

                    step_data = focus_df[focus_df['time'] >= (step_start + datetime.timedelta(seconds=FOCUS_PIEZO_SETTLING_S))]
                    if is_focus_good(step_data):
                        # store data
                        focus_retries = 0
                        # have to re-index to not be working on a copy of the data
                        focus_df.loc[focus_df['time'] >= (step_start + datetime.timedelta(seconds=FOCUS_PIEZO_SETTLING_S)), 'quality'] = 'good'
                        logging.info(f"Step {piezo.code} var: {step_data['focus'].std()}, mean: {step_data['focus'].mean()}, len: {len(step_data['focus'])}")
                        curve_df.loc[len(curve_df)] = [
                            piezo.code,
                            jubilee.z,
                            step_data['focus'].mean(),
                            step_data['focus'].std(),
                            jubilee.z - (piezo.code * PIEZO_UM_PER_LSB) / 1000.0
                        ]
                        # nudge by 5um
                        last_code = piezo.code
                        nudge = FOCUS_STEP_UM / PIEZO_UM_PER_LSB
                        next_code = int(nudge * step_direction + last_code) # positive step direction => negative z-height

                        # step if possible; if not adjust Z-axis. this might be dubious due to z-axis inaccuracy
                        # (may be better to just determine the overall range prior to focus sweep and try to center things up)
                        if piezo.is_code_valid(next_code):
                            piezo.set_code(next_code)
                        else:
                            if next_code > PIEZO_MAX_CODE / 2: # going too high
                                # z-axis has mapping of lower values are closer to objective, so subtract a min step
                                jubilee.step_axis('z', -MIN_JUBILEE_STEP_MM)
                                # recalculate the next code
                                next_code = last_code - (MIN_JUBILEE_STEP_MM * 1000.0) / PIEZO_UM_PER_LSB
                                piezo.set_code(next_code)
                            else: # going too low
                                jubilee.step_axis('z', MIN_JUBILEE_STEP_MM)
                                next_code = last_code + (MIN_JUBILEE_STEP_MM * 1000.0) / PIEZO_UM_PER_LSB
                                piezo.set_code(next_code)

                        # report to UI
                        poi = Poi(jubilee.x, jubilee.y, jubilee.z, piezo.code)
                        jubilee_state.put(poi, block=False)
                        # prep next step
                        step_start = datetime.datetime.now()
                        focus_steps += 1
                        if focus_steps > FOCUS_SLOPE_SEARCH_STEPS:
                            focus_state = "ANALYZE_SLOPE"
                    else:
                        focus_df.loc[focus_df['time'] >= (step_start + datetime.timedelta(seconds=FOCUS_PIEZO_SETTLING_S)), 'quality'] = 'bad'
                        step_start = datetime.datetime.now()
                        focus_retries += 1
                        if focus_retries > FOCUS_MAX_RETRIES:
                            logging.error("Data quality too low for autofocus. Aborting run. Check for excessive environment vibration or camera noise.")
                            focus_state = "EXIT"
            elif focus_state == "ANALYZE_SLOPE":
                sorted_curve = curve_df.sort_values(by="total z") # now sorted from lowest to highest total Z (highest to lowest piezo code)
                max_row = sorted_curve.reset_index(drop=True)['focus_mean'].idxmax()
                if max_row >= len(curve_df) - FOCUS_STEPS_MARGIN:
                    # best focus is toward highest total Z
                    #  -> lower the piezo code to increase total Z more
                    if step_direction != -1:
                        piezo.set_code(focus_starting_piezo)
                        jubilee.set_axis('z', focus_starting_z)
                        step_direction = -1
                    else:
                        focus_starting_piezo = piezo.code
                        focus_starting_z = jubilee.z
                    focus_steps = 0
                    step_start = datetime.datetime.now()
                    focus_state = "FIND_SLOPE"
                elif max_row < FOCUS_STEPS_MARGIN:
                    # best focus is toward lowest total Z
                    #  -> raise the piezo code to reduce totalZ more
                    if step_direction != 1:
                        piezo.set_code(focus_starting_piezo)
                        jubilee.set_axis('z', focus_starting_z)
                        step_direction = 1
                    else:
                        focus_starting_piezo = piezo.code
                        focus_starting_z = jubilee.z
                    focus_steps = 0
                    step_start = datetime.datetime.now()
                    focus_state = "FIND_SLOPE"
                else:
                    # fit to 2nd order and find maxima
                    coefficients = np.polyfit(curve_df['total z'], curve_df['focus_mean'], deg=2)
                    logging.info(curve_df['total z'])
                    logging.info(curve_df['focus_mean'])
                    logging.info(f"coefficients: {coefficients}")
                    z_maxima = -coefficients[1] / (2 * coefficients[0])
                    if z_maxima < curve_df['total z'].min() - AUTOFOCUS_SAFETY_MARGIN_MM \
                    or z_maxima > curve_df['total z'].max() + AUTOFOCUS_SAFETY_MARGIN_MM:
                        logging.error(f"Computed focus is bogus: {z_maxima:0.3f}, [{curve_df['total z'].min():0.3f}, {curve_df['total z'].max():0.3f}]")
                    else:
                        # servo machine to maxima
                        logging.info(f"Focus point at {z_maxima:0.3f}mm")
                        (new_z, new_piezo) = partition_z(z_maxima, jubilee.z)
                        jubilee.set_axis('z', new_z)
                        piezo.set_code(new_piezo)

                    # check focus score
                    focus_state = "EXIT"

            elif focus_state == "EXIT":
                # Call this when we're done with fine focus algo...
                fine_focus_event.clear()
                focus_state = "IDLE"
                focus_df.to_csv("focus.csv")
                curve_df.to_csv("curve.csv")
            else:
                logging.warning("Unrecognized focus state: f{focus_state}")

        # Hardware control loop
        msg = midi.midi.poll()
        if msg is None:
            # Only update the motor value after the controller is stable for one cycle
            if local_angle_changed:
                if new_local_angle_control_value == last_local_angle_control_value:
                    light.set_angle_from_control(new_local_angle_control_value, 'local')
                    local_angle_changed = False
                last_local_angle_control_value = new_local_angle_control_value
            if remote_angle_changed:
                if new_remote_angle_control_value == last_remote_angle_control_value:
                    light.set_angle_from_control(new_remote_angle_control_value, 'remote')
                    remote_angle_changed = False
                last_remote_angle_control_value = new_remote_angle_control_value
            if rotation_changed:
                if new_rotation_value == last_rotation_value:
                    r = (float(new_rotation_value) / 127.0) * \
                            (CONTROL_MAX_ROTATION_ANGLE - CONTROL_MIN_ROTATION_ANGLE) + CONTROL_MIN_ROTATION_ANGLE
                    jubilee.set_rotation(r)
                    rotation_changed = False
                last_rotation_value = new_rotation_value
            if piezo_changed:
                if new_piezo_value == last_piezo_value:
                    nudge = float(new_piezo_value) * (MAX_PIEZO / 127.0)
                    piezo.set_code(int(nudge))
                    piezo_changed = False
                    poi = Poi(jubilee.x, jubilee.y, jubilee.z, piezo.code)
                    jubilee_state.put(poi, block=False)
                last_piezo_value = new_piezo_value

            if profiling:
                logging.info(f"poll {datetime.datetime.now() - start}")
            time.sleep(0.01) # yield our quantum
            continue
        # hugely inefficient O(n) search on every iter, but "meh"
        # that's why we have Moore's law.
        valid = False
        for (name, control_node) in schema.items():
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
                        local_angle_changed = True
                        new_local_angle_control_value = float(control_value)
                    elif name == "brightness-local":
                        bright = float(control_value) * (MAX_LIGHT / 127.0)
                        light.set_intensity_local(int(bright))
                    elif name == "angle-remote":
                        remote_angle_changed = True
                        new_remote_angle_control_value = float(control_value)
                    elif name == "brightness-remote":
                        bright = float(control_value) * (MAX_LIGHT / 127.0)
                        light.set_intensity_remote(int(bright))
                    elif name == "nudge-piezo":
                        piezo_changed = True
                        new_piezo_value = float(control_value)
                    elif name == "gamma":
                        last_gamma = (float(control_value) / 127.0) * MAX_GAMMA
                        if gamma_enabled:
                            gamma.gamma = last_gamma 
                        else:
                            gamma.gamma = 1.0
                    elif name == "rotation":
                        rotation_changed = True
                        new_rotation_value = float(control_value)
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
                            if jubilee.is_on():
                                logging.info("Motors are OFF")
                                midi.set_led_state(note_id, False)
                                jubilee.motors_off()
                            else:
                                logging.info("Motors are ON and origin is set")
                                midi.set_led_state(note_id, True)
                                jubilee.motors_on_set_zero()
                        elif name == 'ESTOP button':
                            for (name, val) in curvals.items():
                                refvals[name] = val
                            print("ESTOP hit, exiting")
                            all_leds_off(schema, midi)
                            jubilee.estop()
                            time.sleep(5)
                            cam_quit.set()
                            return
                        elif '1050 button' in name:
                            midi.set_led_state(note_id, light.toggle_1050())
                            light.commit_wavelength()
                        elif 'set POI' in name:
                            if not jubilee.is_on():
                                logging.warning("Please turn Jubilee on before setting a POI")
                                continue
                            maybe_poi = re.findall(r'^set POI (\d)', name)
                            if len(maybe_poi) == 1:
                                midi.set_led_state(note_id, True)
                                jubilee.set_poi(int(maybe_poi[0]), piezo.code)
                                logging.info(f"Set POI {maybe_poi[0]}. X: {jubilee.x:0.2f}, Y: {jubilee.y:0.2f}, Z: {jubilee.z:0.2f}, P: {piezo.code}, Z': {total_z_mm(jubilee.z, piezo.code):0.3f}")
                        elif 'recall POI' in name:
                            maybe_poi = re.findall(r'^recall POI (\d)', name)
                            if len(maybe_poi) == 1:
                                poi_index = int(maybe_poi[0])
                                piezo_residual = jubilee.recall_poi(poi_index)
                                if piezo_residual is not None:
                                    piezo.set_code(piezo_residual)
                                    logging.info(f"Recall POI {poi_index}. X: {jubilee.x:0.2f}, Y: {jubilee.y:0.2f}, Z: {jubilee.z:0.2f}, P: {piezo.code}, Z': {total_z_mm(jubilee.z, piezo.code):0.3f}")
                                    poi = Poi(jubilee.x, jubilee.y, jubilee.z, piezo.code)
                                    jubilee_state.put(poi, block=False)

                        elif name == 'quit button':
                            jubilee.motors_off()
                            all_leds_off(schema, midi)
                            logging.info("Quitting controller...")
                            cam_quit.set()
                            return
                        elif name == 'gamma button':
                            if gamma_enabled:
                                midi.set_led_state(note_id, False)
                                gamma_enabled = False
                                gamma.gamma = 1.0
                            else:
                                midi.set_led_state(note_id, True)
                                gamma_enabled = True
                                gamma.gamma = last_gamma
                        elif name == 'report position button':
                            try:
                                logging.info(f"X: {jubilee.x:0.2f}, Y: {jubilee.y:0.2f}, Z: {jubilee.z:0.2f}, P: {piezo.code}, Z': {total_z_mm(jubilee.z, piezo.code):0.3f}")
                            except:
                                logging.info("Machine is IDLE or controls not synchronized")
                        elif name == 'automate button':
                            if args.automation == 'step':
                                # check that the POIs have been set
                                if jubilee.poi[0] is None or jubilee.poi[1] is None:
                                    logging.warning("POIs have not been set, can't automate!")
                                    continue
                                if jubilee.poi[2] is None:
                                    # simplify edge cases by just duplicating to create our third POI
                                    jubilee.poi[2] = jubilee.poi[1]
                                # plan the path of steps
                                min_x = min([i.x for i in jubilee.poi])
                                max_x = max([i.x for i in jubilee.poi])
                                min_y = min([i.y for i in jubilee.poi])
                                max_y = max([i.y for i in jubilee.poi])

                                x_path = np.arange(min_x, max_x, stepsize)
                                if len(x_path) == 0: # this happens if min == max
                                    x_path = [min_x]
                                # make sure we include the end interval in the photo region
                                if max(x_path) < max_x:
                                    x_path = np.append(x_path, max(x_path) + stepsize)
                                # These fail due to floating point rounding errors :P
                                # assert(max(x_path) >= max_x)
                                y_path = np.arange(min_y, max_y, stepsize)
                                if len(y_path) == 0: # handle min == max
                                    y_path = [min_y]
                                # make sure we include the end interval in the photo region
                                if max(y_path) < max_y:
                                    y_path = np.append(y_path, max(y_path) + stepsize)
                                # assert(max(y_path) >= max_y)

                                # derive Z-plane equation
                                p = []
                                for i in range(3):
                                    p += [np.array((jubilee.poi[i].x, jubilee.poi[i].y, total_z_mm(jubilee.poi[i].z, jubilee.poi[i].piezo)))]

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

                                for x in x_path:
                                    for y in y_path:
                                        # derive composite-z
                                        zp = -(a * x + b * y + d) / c

                                        # Compute Z partition
                                        (z, p_lsb) = partition_z(zp, jubilee.z)

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
                                        jubilee.goto((x, y, z))
                                        piezo.set_code(p_lsb)

                                        # setup the image_name object
                                        image_name.x = jubilee.x
                                        image_name.y = jubilee.y
                                        image_name.z = jubilee.z
                                        image_name.p = piezo.code
                                        image_name.i = light.intensity_local
                                        image_name.t = light.angle_local
                                        image_name.j = light.intensity_remote
                                        image_name.u = light.angle_remote
                                        image_name.a = jubilee.r
                                        image_name.cur_rep = None
                                        image_name.rep = args.reps
                                        # wait for system to settle
                                        logging.info(f"settling for {args.settling}s")
                                        time.sleep(args.settling)
                                        # this should trigger the picture
                                        logging.info(f"triggering photos")
                                        auto_snap_event.set()
                                        auto_snap_done.wait()
                                        logging.info(f"got photo done event")
                                        # reset the flags
                                        auto_snap_event.clear()
                                        auto_snap_done.clear()

                            elif args.automation == 'psi': # rotate the light around the vertical axis
                                psi_base = jubilee.get_rotation()
                                for delta_psi in range(0, 95, 1):
                                    jubilee.set_rotation(psi_base + delta_psi)
                                    image_name.x = jubilee.x
                                    image_name.y = jubilee.y
                                    image_name.z = jubilee.z
                                    image_name.p = piezo.code
                                    image_name.i = light.intensity_local
                                    image_name.t = light.angle_local
                                    image_name.j = light.intensity_remote
                                    image_name.u = light.angle_remote
                                    image_name.a = jubilee.r
                                    image_name.cur_rep = None
                                    image_name.rep = args.reps
                                    # wait for system to settle
                                    logging.info(f"settling for {args.settling}s")
                                    time.sleep(args.settling)
                                    # this should trigger the picture
                                    logging.info(f"triggering photos")
                                    auto_snap_event.set()
                                    auto_snap_done.wait()
                                    logging.info(f"got photo done event")
                                    # reset the flags
                                    auto_snap_event.clear()
                                    auto_snap_done.clear()

                            elif args.automation == 'theta': # rotate the light around the vertical axis
                                for theta in range(0, 127, 2):
                                    light.set_angle_from_control(theta, 'remote')
                                    image_name.x = jubilee.x
                                    image_name.y = jubilee.y
                                    image_name.z = jubilee.z
                                    image_name.p = piezo.code
                                    image_name.i = light.intensity_local
                                    image_name.t = light.angle_local
                                    image_name.j = light.intensity_remote
                                    image_name.u = light.angle_remote
                                    image_name.a = jubilee.r
                                    image_name.cur_rep = None
                                    image_name.rep = args.reps
                                    # wait for system to settle
                                    logging.info(f"settling for {args.settling}s")
                                    time.sleep(args.settling)
                                    # this should trigger the picture
                                    logging.info(f"triggering photos")
                                    auto_snap_event.set()
                                    auto_snap_done.wait()
                                    logging.info(f"got photo done event")
                                    # reset the flags
                                    auto_snap_event.clear()
                                    auto_snap_done.clear()

                            else:
                                logging.error("Unrecognized automation type, doing nothing.")
                            
                            logging.info("Automation done!")

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

                    if jubilee.step_axis(name, step):
                        logging.warning("Motor command timeout!")
                        midi.clear_events()
            # update the UI thread
            poi = Poi(jubilee.x, jubilee.y, jubilee.z, piezo.code)
            jubilee_state.put(poi, block=False)

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
    args = parser.parse_args()

    if args.mag == 5:
        stepsize = 0.5
    elif args.mag == 10:
        stepsize = 0.3
    elif args.mag == 20:
        stepsize = 0.1
    else:
        logging.error("Magnification must be one of [5, 10, 20]")
        exit(0)

    gamma = Gamma()
    image_name = ImageNamer()
    image_name.name = args.name
    image_name.rep = args.reps
    auto_snap_event = Event()
    auto_snap_done = Event()
    fine_focus_event = Event()
    focus_score = queue.Queue()
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
                    q = Thread(target=quitter, args=[jubilee, light, piezo, cam_quit])
                    q.start()
                    l = Thread(target=loop, args=[
                        args, stepsize, j, m, l, p, gamma, image_name,
                        auto_snap_done, auto_snap_event, schema,
                        focus_score, jubilee_state, fine_focus_event
                    ])
                    l.start()
                    # when the quitter exits, everything has been brought down in an orderly fashion
                    q.join()
                    # inform the snapper thread to quit
                    image_name.quit = True
                    auto_snap_event.set()
                    logging.debug("Midi control thread reached quit")

if __name__ == "__main__":
    main()

