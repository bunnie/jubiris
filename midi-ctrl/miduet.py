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
#  - python3 -m pip install numpy opencv-python-headless qimage2ndarray
# The headless version of opencv-python is needed because it bundles a Qt plugin that is the wrong version
#
# Helpful: https://docs.google.com/document/d/1zeRPklp_Mo_XzJZUKu2i-p1VgfBoUWF0JKZ5CzX8aB0/edit#heading=h.pb5rw66aumst
# ^^^ "Unofficial Communications Protocol for Akai MidiMix controller"

import argparse
import serial
import mido
import json
import time
import pprint
import logging
import re

from cam import cam
from threading import Thread, Event
import numpy as np
import math

SCHEMA_STORAGE = 'miduet-config.json'
SCHEMA_VERSION = 2
MAX_POI = 3
MIN_ANGLE = -100
MAX_ANGLE = 100
MAX_LIGHT = 4096
MAX_PIEZO = 8192 #16383
MAX_GAMMA = 2.0

PIEZO_UM_PER_VOLT = 1.17 # estimated
PIEZO_MAX_VOLT = 103.97
PIEZO_MAX_CODE = 16383
PIEZO_UM_PER_LSB = 0.007425

cam_quit = Event()

class Light:
    def __init__(self, ports=[]):
        self.port = None
        self.intensity = 0
        self.angle = 0
        for port in ports:
            try:
                ser = serial.Serial(port, 115200, timeout = 1.0)
                logging.debug(f"Trying port {port} for light")
                ser.write(bytes("?\n", 'utf-8'))
                line = ''
                while True:
                    c = ser.read(1)
                    if len(c) == 0:
                        break
                    line += c.decode('utf-8')
                    if 'OK\n' in line:
                        break
                if "V1 LED" in line:
                    logging.debug(f"Found V1 LED on {port}")
                    self.ser = ser
                    self.port = port
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
    
    def set_intensity(self, i):
        i = int(i)
        if i > MAX_LIGHT:
            logging.warning(f"requested intensity too great {i}")
            return
        if i < 0:
            logging.warning(f"requested intensity out of range {i}")
            return
        self.send_cmd(f"I {i}")
        self.intensity = i

    def set_angle(self, a):
        a = int(a)
        if a > MAX_ANGLE:
            logging.warning(f"requested angle too great {a}")
            return
        if a < MIN_ANGLE:
            logging.warning(f"requested angle too small {a}")
            return
        self.send_cmd(f"A {a}")
        self.angle = a

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.ser.is_open:
            self.send_cmd("I 0\n")
            self.intensity = 0
            self.send_cmd("A 0\n")
            self.angle = 0
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
    def __init__(self, x, y, z, theta, i, piezo):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.i = i
        self.piezo = piezo

class Jubilee:
    def __init__(self, port):
        self.port = port
        try:
            ser = serial.Serial(port, 115200, timeout = 1.0)
            logging.debug("Duet3D serial port opened")
            self.ser = ser
            self.motors_off()
            self.poi = [None] * MAX_POI # this is an array of POI objects
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

    def motors_off(self):
        self.x = None
        self.y = None
        self.z = None
        self.state = 'OFF'
        return self.send_cmd('M18')
    
    def is_on(self):
        return self.state == 'ON'
    
    def motors_on_set_zero(self):
        self.send_cmd('G92 X0 Y0 Z10 U0') # declare this position as origin
        self.x = 0.0
        self.y = 0.0
        self.z = 10.0
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
        self.sleep(8) # enough time to boot?? TODO: there is probably a smarter way to watch for the boot condition
        return self.send_cmd('M115')

    def set_poi(self, index, theta, i, piezo):
        if not self.is_on():
            return
        # index should be a number from 1 through MAX_POI, inclusive
        if index > MAX_POI or index == 0:
            return
        logging.debug(f"POI {index} set to {self.x}, {self.y}, {self.z}")
        self.poi[index - 1] = Poi(self.x, self.y, self.z, theta, i, piezo)

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
        return (poi.theta, poi.i, poi.piezo)
            
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
        self.rep = None # repetitions of the same spot
        self.cur_rep = None
        self.quit = False # flag to indicate if we're exiting
    def is_init(self):
        return self.name is not None and self.x is not None and self.y is not None \
        and self.z is not None and self.p is not None \
        and self.i is not None and self.t is not None
    def get_name(self):
        if self.cur_rep is not None:
            r = str(self.cur_rep)
        else:
            r = '1'
        return f'x{self.x:0.2f}_y{self.y:0.2f}_z{self.z:0.2f}_p{self.p}_i{self.i}_t{self.t}_r{r}'

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

def loop(args, jubilee, midi, light, piezo, gamma, image_name, auto_snap_done, auto_snap_event, schema):
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
    last_angle = None
    last_intensity = None
    last_piezo = None
    
    #for msg in midi.midi:
    while True:
        if cam_quit.is_set():
            return
        msg = midi.midi.poll()
        if msg is None:
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
                    if name == "angle-light":
                        angle = (float(control_value) * (MAX_ANGLE - MIN_ANGLE) / 127.0) + float(MIN_ANGLE)
                        light.set_angle(int(angle))
                    elif name == "brightness-light":
                        bright = float(control_value) * (MAX_LIGHT / 127.0)
                        light.set_intensity(int(bright))
                    elif name == "nudge-piezo":
                        nudge = float(control_value) * (MAX_PIEZO / 127.0)
                        piezo.set_code(int(nudge))
                    elif name == "gamma":
                        last_gamma = (float(control_value) / 127.0) * MAX_GAMMA
                        if gamma_enabled:
                            gamma.gamma = last_gamma 
                        else:
                            gamma.gamma = 1.0
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
                        elif 'set POI' in name:
                            maybe_poi = re.findall(r'^set POI (\d)', name)
                            if len(maybe_poi) == 1:
                                midi.set_led_state(note_id, True)
                                jubilee.set_poi(int(maybe_poi[0]), light.angle, light.intensity, piezo.code)
                                logging.info(f"Set POI {maybe_poi[0]}. X: {jubilee.x:0.2f}, Y: {jubilee.y:0.2f}, Z: {jubilee.z:0.2f}, P: {piezo.code}, Z': {(jubilee.z + piezo.code * PIEZO_UM_PER_LSB / 1000):0.3f}, I: {light.intensity}, A: {light.angle}")
                        elif 'recall POI' in name:
                            maybe_poi = re.findall(r'^recall POI (\d)', name)
                            if len(maybe_poi) == 1:
                                poi_index = int(maybe_poi[0])
                                maybe_poi = jubilee.recall_poi(poi_index)
                                if maybe_poi is not None:
                                    (l_t, l_i, l_p) = maybe_poi
                                    if l_t is not None:
                                        light.set_angle(l_t)
                                    if l_i is not None:
                                        light.set_intensity(l_i)
                                    if l_p is not None:
                                        piezo.set_code(l_p)
                                    logging.info(f"Recall POI {poi_index}. X: {jubilee.x:0.2f}, Y: {jubilee.y:0.2f}, Z: {jubilee.z:0.2f}, P: {piezo.code}, Z': {(jubilee.z + piezo.code * PIEZO_UM_PER_LSB / 1000):0.3f}, I: {light.intensity}, A: {light.angle}")

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
                                logging.info(f"X: {jubilee.x:0.2f}, Y: {jubilee.y:0.2f}, Z: {jubilee.z:0.2f}, P: {piezo.code}, Z': {(jubilee.z + piezo.code * PIEZO_UM_PER_LSB / 1000):0.3f}, I: {light.intensity}, A: {light.angle}")
                            except:
                                logging.info("Machine is IDLE or controls not synchronized")
                        elif name == 'automate button':
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
                            y_path = np.arange(min_y, max_y, args.stepsize)
                            if len(y_path) == 0:
                                y_path = [min_y]

                            # derive Z-plane equation
                            p = []
                            for i in range(3):
                              p += [np.array((jubilee.poi[i].x, jubilee.poi[i].y, jubilee.poi[i].z + jubilee.poi[i].piezo  * PIEZO_UM_PER_LSB / 1000))]

                            if np.array_equal(p[1], p[2]):
                                p[2][0] += 0.001 # perturb the second point slightly to make the equations solvable in case only two POI set

                            v1 = p[1] - p[0]
                            v2 = p[2] - p[0]
                            normal_vector = np.cross(v1, v2)
                            a, b, c = normal_vector
                            d = -(a * p[0][0] + b * p[0][1] + c * p[0][2])

                            # TODO: interpolate angle and light. For now, just use the setting at POI[0]
                            light.set_intensity(jubilee.poi[0].i)
                            light.set_angle(jubilee.poi[0].theta)

                            logging.info(f"stepping x {x_path}")
                            logging.info(f"stepping y {y_path}")

                            for x in x_path:
                                for y in y_path:
                                    # derive composite-z
                                    zp = -(a * x + b * y + d) / c
                                    # get jubilee to 0.02mm below the target Z
                                    Z_INCREMENT = 1/0.02
                                    z = math.floor(zp * Z_INCREMENT) / Z_INCREMENT
                                    # make up the rest with the piezo
                                    p_lsb = ((zp - z) * 1000) / PIEZO_UM_PER_LSB
                                    if p_lsb > PIEZO_MAX_CODE:
                                        logging.warning(f"Piezo vale out of bounds: {p_lsb}, aborting step")
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
                                    image_name.p = piezo.code,
                                    image_name.i = light.intensity,
                                    image_name.t = light.angle,
                                    image_name.cur_rep = None
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
                        step = step / 5.0
                    logging.debug(f"Delta of {delta} for {name}")

                    if jubilee.step_axis(name, step):
                        logging.warning("Motor command timeout!")
                        midi.clear_events()

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
        "angle-light" : None,
        "brightness-light" : None,
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
        'quit button': None,
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
        "--duet-port", required=False, help="path to duet3d serial port", default='/dev/ttyACM0'
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
    if not args.no_cam:
        c = Thread(target=cam, args=[cam_quit, gamma, image_name, auto_snap_event, auto_snap_done])
        c.start()

    numeric_level = getattr(logging, args.loglevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % args.loglevel)
    logging.basicConfig(level=numeric_level)
    # TODO: turn this into a command line argument
    iris_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']

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
    jubilee = Jubilee(args.duet_port)
    midi = Midi(midimix_name)
    light = Light(iris_ports)
    piezo = Piezo(iris_ports)
    logging.info("MIDI-to-Jubilee Controller Starting. Motors are IDLE.")
    logging.info("---> Please press SEND ALL to initialize <---")
    # wrap in 'with' so we can shut things down on exit if necessary
    with jubilee as j:
        with midi as m:
            with light as l:
                with piezo as p:
                    q = Thread(target=quitter, args=[jubilee, light, piezo, cam_quit])
                    q.start()
                    l = Thread(target=loop, args=[
                        args, j, m, l, p, gamma, image_name, auto_snap_done, auto_snap_event, schema
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

