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

import iqmotion as iq

import os
import cv2
envpath = '/home/bunnie/.local/lib/python3.10/site-packages/cv2/qt/plugins/platforms'
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = envpath

SERIAL_NO_LED   = 'FTCYSOO2'
SERIAL_NO_PIEZO = 'FTCYSQ4J'
SERIAL_NO_MOT0  = 'FTD3MKXS'
SERIAL_NO_MOT1  = 'FTD3MLD8'

SCHEMA_STORAGE = 'miduet-config.json'
SCHEMA_VERSION = 3
MAX_POI = 3
MIN_ROTATION_ANGLE = -100.0
MAX_ROTATION_ANGLE = 100.0
CONTROL_MIN_ROTATION_ANGLE = -80.0 # somewhat not exactly degrees somehow...
CONTROL_MAX_ROTATION_ANGLE = 80.0
MAX_LIGHT = 4096
MAX_PIEZO = 8192 #16383
MAX_GAMMA = 2.0

PIEZO_UM_PER_VOLT = 1.17 # estimated
PIEZO_MAX_VOLT = 103.97
PIEZO_MAX_CODE = 16383
PIEZO_UM_PER_LSB = 0.007425

cam_quit = Event()

def send_tranjectory(iq_module, time_cmd, angle_cmd):
    iq_module.set(
        "multi_turn_angle_control", "trajectory_angular_displacement", angle_cmd
    )
    iq_module.set("multi_turn_angle_control", "trajectory_duration", time_cmd)

class Light:
    def __init__(self, light, mot_local, mot_remote):
        self.port = None
        self.intensity_local = 0
        self.intensity_remote = 0
        self.angle_local = 64
        self.angle_remote = 64
        self.led_1050_on = True
        try:
            ser = serial.Serial(light, 115200, timeout = 1.0)
            logging.debug(f"Trying port {light} for light")
            ser.write(bytes("?\n", 'utf-8'))
            line = ''
            while True:
                c = ser.read(1)
                if len(c) == 0:
                    break
                line += c.decode('utf-8')
                if 'OK\n' in line:
                    break
            if "V2 LED" in line:
                logging.debug(f"Found V2 LED on {light}")
                self.ser = ser
                self.port = light
            else:
                ser.close()
        except serial.SerialException as e:
            logging.error(f"Serial error: {e}")
            return None
        if self.port is None:
            logging.error(f"Couldn't find LED controller on {light}")
            exit(1)
        self.commit_wavelength()
        self.com = {}
        self.iq = {}
        self.cur_angle = {}
        self.com['local'] = iq.SerialCommunicator(mot_local)
        self.iq['local'] = iq.ServoModule(self.com['local'], 0)
        self.com['remote'] = iq.SerialCommunicator(mot_remote)
        self.iq['remote'] = iq.ServoModule(self.com['remote'], 0)

        self.upper_limit = {}
        self.lower_limit = {}
        self.get_limit_switches()
        self.upper_angle_limit = {}
        self.lower_angle_limit = {}

        logging.info("Homing remote light...")
        self.home_angle('remote')
        logging.info("Homing local light...")
        self.home_angle('local')

    def read_angle(self, which):
        return self.iq[which].get("multi_turn_angle_control", "obs_angular_displacement")
    def nudge_angle(self, which, how_much, movement_time=0.1):
        self.cur_angle[which] = self.read_angle(which)
        new_angle = self.cur_angle[which] + how_much
        send_tranjectory(self.iq[which], movement_time, new_angle)
        time.sleep(movement_time)
        self.cur_angle[which] = self.read_angle(which)
    def set_angle(self, which, angle, movement_time=0.2):
        MAX_SLEW = 5 * math.pi / 0.2
        if angle < self.lower_angle_limit[which] or angle > self.upper_angle_limit[which]:
            logging.error(f"Requested angle {angle} is out of bounds, doing nothing!")
            self.cur_angle[which] = self.read_angle(which)
            return
        
        # limit the slew rate of the motor
        slew = abs(self.cur_angle[which] - angle) / movement_time # rads/s
        if slew > MAX_SLEW:
            movement_time = abs(self.cur_angle[which] - angle) / MAX_SLEW

        send_tranjectory(self.iq[which], movement_time, angle)
        time.sleep(movement_time)
        self.cur_angle[which] = self.read_angle(which)

    def other_motor(self, which):
        if which == 'local':
            return 'remote'
        elif which == 'remote':
            return 'local'
        else:
            logging.error("invalid motor specifier, abort!")
            exit(1)

    def home_angle(self, which):
        INCREMENT = math.pi
        #while True:
        #    self.get_limit_switches()
        self.get_limit_switches()
        if self.lower_limit[which] and self.upper_limit[which] or \
            self.lower_limit[self.other_motor(which)] and self.upper_limit[self.other_motor(which)]:
            logging.error("Both upper and lower limit switches are set, aborting homing!")
            return
        
        # if a limit switch is currently hit, back us off
        adjusted = False
        if self.lower_limit[which]:
            adjusted = True
            self.nudge_angle(which, INCREMENT * 3)
        elif self.upper_limit[which]:
            adjusted = True
            self.nudge_angle(which, -INCREMENT * 3)
        elif self.lower_limit[self.other_motor(which)]:
            adjusted = True
            self.nudge_angle(self.other_motor(which), INCREMENT * 3)
        elif self.upper_limit[which]:
            adjusted = True
            self.nudge_angle(self.other_motor[which], -INCREMENT * 3)
        if adjusted:
            time.sleep(0.5)
            self.get_limit_switches()
        
        # search down
        while not self.upper_limit[which] and not self.lower_limit[which] and \
            not self.upper_limit[self.other_motor(which)] and not self.lower_limit[self.other_motor(which)]:
            self.nudge_angle(which, -INCREMENT)
            self.get_limit_switches()
        assert self.lower_limit[which], "wrong limit switch hit!"
        self.lower_angle_limit[which] = self.read_angle(which)

        # back off the lower limit
        self.nudge_angle(which, INCREMENT * 34, movement_time=1)
        # let limit switch rebound
        time.sleep(0.2)
        self.get_limit_switches()

        # search up
        while not self.upper_limit[which] and not self.lower_limit[which] and \
            not self.upper_limit[self.other_motor(which)] and not self.lower_limit[self.other_motor(which)]:
            self.nudge_angle(which, INCREMENT)
            self.get_limit_switches()
        assert self.upper_limit[which], "wrong limit switch hit!"
        self.upper_angle_limit[which] = self.read_angle(which)

        # center it
        center_angle = (self.upper_angle_limit[which] + self.lower_angle_limit[which]) / 2
        self.set_angle(which, center_angle, movement_time=1.0)
    
    # syncs the local wavelength setting to the controller
    def commit_wavelength(self):
        arg = 0
        if self.led_1050_on:
            self.send_cmd("L")
        else:
            self.send_cmd("H")

    def toggle_1050(self):
        if self.led_1050_on:
            self.led_1050_on = False
        else:
            self.led_1050_on = True
        return self.led_1050_on
    def is_1050_set(self):
        return self.led_1050_on
    def set_1050(self, state):
        self.set_1050 = state

    def send_cmd(self, cmd, timeout=1.0):
        cmd_bytes = bytes(cmd + '\n', 'utf-8')
        logging.debug(str(cmd_bytes, 'utf-8'))
        self.ser.write(cmd_bytes)
        line = ''
        timeout = True
        while True:
            c = self.ser.read(1)
            if len(c) == 0:
                break
            line += c.decode('utf-8')
            if 'OK\n' in line:
                timeout = False
                break

        logging.debug(line)
        return timeout
    
    def set_angle_from_control(self, a, which):
        angle = (a / 127.0) * (self.upper_angle_limit[which] - self.lower_angle_limit[which]) + self.lower_angle_limit[which]
        if which == 'local':
            self.angle_local = int(a)
        elif which == 'remote':
            self.angle_remote = int(a)
        else:
            logging.error(f"Incorrect specifier on angle: {which}, ignoring!")
            return
        self.set_angle(which, angle)

    def set_intensity_local(self, i):
        i = int(i)
        if i > MAX_LIGHT:
            logging.warning(f"requested intensity too great {i}")
            return
        if i < 0:
            logging.warning(f"requested intensity out of range {i}")
            return
        self.send_cmd(f"I {i}")
        self.intensity_local = i

    def set_intensity_remote(self, i):
        i = int(i)
        if i > MAX_LIGHT:
            logging.warning(f"requested intensity too great {i}")
            return
        if i < 0:
            logging.warning(f"requested intensity out of range {i}")
            return
        self.send_cmd(f"R {i}")
        self.intensity_remote = i

    def get_limit_switches(self):
        self.ser.write(bytes("S\n", 'utf-8'))
        line = ''
        while True:
            c = self.ser.read(1)
            if len(c) == 0:
                break
            line += c.decode('utf-8')
            if '\n' in line:
                break
        sws = line.strip().split(',')
        errs = 0
        for sw in sws:
            if sw[:2] == 'LL':
                self.lower_limit['local'] = sw[2] == '0'
            elif sw[:2] == 'LU':
                self.upper_limit['local'] = sw[2] == '0'
            elif sw[:2] == 'RL':
                self.lower_limit['remote'] = sw[2] == '0'
            elif sw[:2] == 'RU':
                self.upper_limit['remote'] = sw[2] == '0'
            else:
                logging.error("Limit switch readout corrupt!")
                time.sleep(0.1)
                if errs > 2:
                    logging.error("Too many errors, quitting.")
                    exit(0)
                errs += 1
        # clear the 'OK\n'
        line = ''
        while True:
            c = self.ser.read(1)
            if len(c) == 0:
                break
            line += c.decode('utf-8')
            if '\n' in line:
                break

        logging.debug(f"upper sw: {self.upper_limit}, lower sw: {self.lower_limit}")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.ser.is_open:
            self.send_cmd("I 0")
            self.intensity_local = 0
            self.send_cmd("R 0")
            self.intensity_remote = 0
            # park the emitters near the bottom
            self.set_angle('local', self.lower_angle_limit['local'] + 3 * math.pi, movement_time=1.5)
            self.set_angle('remote', self.lower_angle_limit['remote'] + 3 * math.pi, movement_time=1.5)
            self.ser.close()
            logging.debug("Serial port closed")

class Piezo:
    def __init__(self, ports=[]):
        self.port = None
        self.code = 0
        self.enabled = False
        for port in ports:
            try:
                ser = serial.Serial(port, 115200, timeout = 1.0)
                logging.debug(f"Trying port {port} for piezo")
                ser.write(bytes("?\n", 'utf-8'))
                line = ''
                while True:
                    c = ser.read(1)
                    if len(c) == 0:
                        break
                    line += c.decode('utf-8')
                    if 'OK\n' in line:
                        break
                if "V1 PIEZO" in line:
                    logging.debug(f"Found V1 PIEZO on {port}")
                    self.ser = ser
                    self.port = port
                    self.send_cmd("E") # enable the HV driver
                    self.enabled = True
                    break
                else:
                    ser.close()
            except serial.SerialException as e:
                logging.error(f"Serial error: {e}")
                return None

    def send_cmd(self, cmd, timeout=1.0):
        cmd_bytes = bytes(cmd + '\n', 'utf-8')
        logging.debug(str(cmd_bytes, 'utf-8'))
        self.ser.write(cmd_bytes)
        line = ''
        timeout = True
        while True:
            c = self.ser.read(1)
            if len(c) == 0:
                break
            line += c.decode('utf-8')
            if 'OK\n' in line:
                timeout = False
                break

        logging.debug(line)
        return timeout
    
    def set_code(self, a):
        a = int(a)
        if not self.enabled:
            logging.warning(f"Piezo drivers not enabled")
            return
        if a < 0:
            logging.warning(f"Invalid piezo value {a}")
            return
        if a > PIEZO_MAX_CODE:
            logging.warning(f"Invalid piezo value {a}")
            return
        self.send_cmd(f"A {a}")
        self.code = a
    
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.ser.is_open:
            self.send_cmd("A 0")
            self.code = 0
            self.send_cmd("D")
            self.enabled = False
            self.ser.close()
            logging.debug("Serial port closed")

class Poi:
    def __init__(self, x, y, z, piezo):
        self.x = x
        self.y = y
        self.z = z
        self.piezo = piezo

class Jubilee:
    def __init__(self, port):
        self.port = port
        self.state = 'OFF'
        try:
            ser = serial.Serial(port, 115200, timeout = 1.0)
            logging.debug("Duet3D serial port opened")
            self.ser = ser
            self.motors_off()
            self.poi = [None] * MAX_POI # this is an array of POI objects
        except serial.SerialException as e:
            logging.error(f"Serial error: {e}")
            return None
    
    def update_port(self, port):
        self.port = port
        try:
            ser = serial.Serial(port, 115200, timeout = 1.0)
            logging.info(f"Duet3D serial port updated to {port}")
            self.ser = ser
        except serial.SerialException as e:
            logging.error(f"Serial error: {e}")
            return None

    def __enter__(self):
        return self

    def send_cmd(self, cmd, timeout=1.0):
        cmd_bytes = bytes(cmd + '\n', 'utf-8')
        logging.debug(str(cmd_bytes, 'utf-8'))
        self.ser.write(cmd_bytes)
        line = ''
        timeout = True
        while True:
            c = self.ser.read(1)
            if len(c) == 0:
                break
            line += c.decode('utf-8')
            if 'ok\n' in line:
                timeout = False
                break

        logging.debug(line)
        return timeout

    def step_axis(self, axis, step):
        if not self.is_on():
            return False # don't return an error, just silently fail
        if step == 0.0:
            return False

        if 'x' in axis:
            self.x += step
        elif 'y' in axis:
            self.y += step
        elif 'z' in axis:
            self.z += step
        else:
            print('axis not yet implemented')
        return self.sync_to_mach()
    
    def get_rotation(self):
        return self.r
    
    def set_rotation(self, degrees):
        if self.is_on():
            if degrees < MIN_ROTATION_ANGLE or degrees > MAX_ROTATION_ANGLE:
                logging.error(f"requested rotation {degrees} is out of bounds, ignoring")
                return None
            self.r = degrees
            return self.send_cmd(f'G1 V{self.r:0.2f} H4')  # terminates movement on endstop
        else:
            return None
    
    def goto(self, xyz_tuple):
        if not self.is_on():
            return False # don't return an error, just silently fail
        (x, y, z) = xyz_tuple
        self.x = x
        self.y = y
        self.z = z
        return self.sync_to_mach()
    
    # takes our self.x/y/z and sends it to the machine. Also handles any global axis swapping
    def sync_to_mach(self):
        # NOTE axis swap is implemented at sync_to_mach
        return self.send_cmd(f'G1 Y{self.x:0.2f} X{self.y:0.2f} Z{self.z:0.2f}')

    def reset(self):
        return self.send_cmd('M999')

    def motors_off(self):
        self.x = None
        self.y = None
        self.z = None
        self.r = None
        self.state = 'OFF'
        return self.send_cmd('M18')
    
    def is_on(self):
        return self.state == 'ON'
    
    def motors_on_set_zero(self):
        self.send_cmd('G92 X0 Y0 Z10 U0 V0') # declare this position as origin
        self.x = 0.0
        self.y = 0.0
        self.z = 10.0
        self.r = 0.0
        self.send_cmd('G21') # millimeters
        self.send_cmd('G90') # absolute
        self.state = 'ON'

    def estop(self):
        self.send_cmd('M112')
        self.x = None
        self.y = None
        self.z = None
        self.state = 'ESTOP'
    
    def restart(self):
        self.send_cmd('M999')
        time.sleep(8) # enough time to boot?? TODO: there is probably a smarter way to watch for the boot condition
        return self.send_cmd('M115')

    def set_poi(self, index, piezo):
        if not self.is_on():
            return
        # index should be a number from 1 through MAX_POI, inclusive
        if index > MAX_POI or index == 0:
            return
        logging.debug(f"POI {index} set to {self.x}, {self.y}, {self.z}")
        self.poi[index - 1] = Poi(self.x, self.y, self.z, piezo)

    def recall_poi(self, index):
        if not self.is_on():
            return None
        # index should be a number from 1 through MAX_POI, inclusive
        if index > MAX_POI or index == 0:
            return None
        try:
            poi = self.poi[index - 1]
            self.x = poi.x
            self.y = poi.y
            self.z = poi.z
        except AttributeError as e:
            logging.debug(f"POI {index} has not been set")
            return None
        logging.debug(f"Recalling to POI {index}: {self.x}, {self.y}, {self.z}")
        self.sync_to_mach()
        return poi.piezo
            
    def __exit__(self, exc_type, exc_value, traceback):
        if self.ser.is_open:
            self.motors_off()
            self.ser.close()
            logging.debug("Serial port closed")

class Midi:
    def __init__(self, port):
        self.port = port
        try:
            self.midi = mido.open_input(port)
            self.midi_out = mido.open_output(port)
        except Exception as e:
            logging.error(f"MIDI open error: {e}")
            return None
    
    def __enter__(self):
        return self
    
    def clear_events(self):
        for msg in self.midi.iter_pending():
            pass
    
    def set_led_state(self, note, state):
        if state:
            v = 127
        else:
            v = 0
        logging.debug(f"setting led {note}, {state}")
        m = mido.Message('note_on', channel=0, note=note, velocity=v)
        self.midi_out.send(m)

    def __exit__(self, exc_type, exc_value, traceback):
        # I think this one cleans up after itself?
        pass

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

def loop(args, jubilee, midi, light, piezo, gamma, image_name, auto_snap_done, auto_snap_event, schema, image_queue, ui_queue):
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

    while True:
        if cam_quit.is_set():
            all_leds_off(schema, midi)
            return

        if False:       
            # Placeholder for IPC piezo code
            try:
                image = image_queue.get(block=False)
            except queue.Empty:
                pass
            else:
                if not ui_queue.full():
                    ui_queue.put(
                        cv2.resize(image, None, None, 0.2, 0.2)
                    )
                pass

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
                last_piezo_value = new_piezo_value

            time.sleep(0) # yield our quantum
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
                                logging.info(f"Set POI {maybe_poi[0]}. X: {jubilee.x:0.2f}, Y: {jubilee.y:0.2f}, Z: {jubilee.z:0.2f}, P: {piezo.code}, Z': {(jubilee.z - piezo.code * PIEZO_UM_PER_LSB / 1000):0.3f}")
                        elif 'recall POI' in name:
                            maybe_poi = re.findall(r'^recall POI (\d)', name)
                            if len(maybe_poi) == 1:
                                poi_index = int(maybe_poi[0])
                                maybe_poi = jubilee.recall_poi(poi_index)
                                if maybe_poi is not None:
                                    l_p = maybe_poi
                                    if l_p is not None:
                                        piezo.set_code(l_p)
                                    logging.info(f"Recall POI {poi_index}. X: {jubilee.x:0.2f}, Y: {jubilee.y:0.2f}, Z: {jubilee.z:0.2f}, P: {piezo.code}, Z': {(jubilee.z - piezo.code * PIEZO_UM_PER_LSB / 1000):0.3f}")

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
                                logging.info(f"X: {jubilee.x:0.2f}, Y: {jubilee.y:0.2f}, Z: {jubilee.z:0.2f}, P: {piezo.code}, Z': {(jubilee.z - piezo.code * PIEZO_UM_PER_LSB / 1000):0.3f}")
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

                                x_path = np.arange(min_x, max_x, args.stepsize)
                                if len(x_path) == 0: # this happens if min == max
                                    x_path = [min_x]
                                # make sure we include the end interval in the photo region
                                if max(x_path) < max_x:
                                    x_path = np.append(x_path, max(x_path) + args.stepsize)
                                # These fail due to floating point rounding errors :P
                                # assert(max(x_path) >= max_x)
                                y_path = np.arange(min_y, max_y, args.stepsize)
                                if len(y_path) == 0: # handle min == max
                                    y_path = [min_y]
                                # make sure we include the end interval in the photo region
                                if max(y_path) < max_y:
                                    y_path = np.append(y_path, max(y_path) + args.stepsize)
                                # assert(max(y_path) >= max_y)

                                # derive Z-plane equation
                                p = []
                                for i in range(3):
                                    p += [np.array((jubilee.poi[i].x, jubilee.poi[i].y, jubilee.poi[i].z - jubilee.poi[i].piezo  * PIEZO_UM_PER_LSB / 1000))]

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
                                        if jubilee.z - zp > 0 and jubilee.z - zp < (PIEZO_MAX_CODE * PIEZO_UM_PER_LSB / 1000):
                                            # see if we can get to zp without adjusting z at all
                                            z = jubilee.z
                                            p_lsb = ((z - zp) * 1000) / PIEZO_UM_PER_LSB
                                            logging.info(f"p_lsb only: {z}, {p_lsb}")
                                        else:
                                            Z_INCREMENT = 1/0.02
                                            z = math.ceil(zp * Z_INCREMENT) / Z_INCREMENT
                                            # make up the rest with the piezo
                                            p_lsb = ((z - zp) * 1000) / PIEZO_UM_PER_LSB
                                            logging.info(f"z and p_lsb: {z}, {p_lsb}")

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
                    
                    # TODO: add the rotation control here

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
        "--stepsize", required=False, type=float, default=0.1, help="Step size in mm for automation"
    )
    parser.add_argument(
        "--settling", required=False, type=float, default=5.0, help="Settling time in seconds between steps"
    )
    args = parser.parse_args()

    gamma = Gamma()
    image_name = ImageNamer()
    image_name.name = args.name
    image_name.rep = args.reps
    auto_snap_event = Event()
    auto_snap_done = Event()
    image_queue = queue.Queue(maxsize=2) # don't buffer too many frames
    ui_queue = queue.Queue(maxsize=2)
    if not args.no_cam:
        c = Thread(target=cam, args=[cam_quit, gamma, image_name, auto_snap_event, auto_snap_done, image_queue, ui_queue])
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
                        args, j, m, l, p, gamma, image_name, auto_snap_done, auto_snap_event, schema, image_queue, ui_queue
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

