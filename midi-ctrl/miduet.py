#!/usr/bin/env python3
# Midi control:
#  - sudo apt-get install python3-pip
#  - sudo apt-get install alsa-tools alsa-utils libasound2-dev
#  - python3 -m pip install mido
#  - python3 -m pip install python-rtmidi  # note, NOT the 'rtmidi' package, it is incompatible
#  - sudo ln -s /usr/lib/x86_64-linux-gnu/alsa-lib/ /usr/lib64/alsa-lib  # fixes a library path issue that might just be a problem with ubuntu 22.04
# Duet3D serial:
#  - python3 -m pip install pyserial

import argparse
import serial
import mido
import json
import time
import pprint
import logging
import re

from cam import cam
from threading import Thread

SCHEMA_STORAGE = 'miduet-config.json'
SCHEMA_VERSION = 2
MAX_POI = 3
MIN_ANGLE = -100
MAX_ANGLE = 100
MAX_LIGHT = 4096
MAX_PIEZO = 8192 #16383

class Light:
    def __init__(self, ports=[]):
        self.port = None
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

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.ser.is_open:
            self.send_cmd("I 0\n")
            self.send_cmd("A 0\n")
            self.ser.close()
            logging.debug("Serial port closed")

class Piezo:
    def __init__(self, ports=[]):
        self.port = None
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

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.ser.is_open:
            self.send_cmd("A 0")
            self.send_cmd("D")
            self.ser.close()
            logging.debug("Serial port closed")

class Jubilee:
    def __init__(self, port):
        self.port = port
        try:
            ser = serial.Serial(port, 115200, timeout = 1.0)
            logging.debug("Duet3D serial port opened")
            self.ser = ser
            self.motors_off()
            self.poi = [()] * MAX_POI # this will be a list of x/y/z tuples that set points of interest
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
            self.send_cmd(f'G1 X{self.x}')
        elif 'y' in axis:
            self.y += step
            self.send_cmd(f'G1 Y{self.y}')
        elif 'z' in axis:
            self.z += step
            self.send_cmd(f'G1 Z{self.z}')
        else:
            print('axis not yet implemented')
    
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

    def set_poi(self, index):
        if not self.is_on():
            return
        # index should be a number from 1 through MAX_POI, inclusive
        if index > MAX_POI or index == 0:
            return
        logging.debug(f"POI {index} set to {self.x}, {self.y}, {self.z}")
        self.poi[index - 1] = (self.x, self.y, self.z)

    def recall_poi(self, index):
        if not self.is_on():
            return
        # index should be a number from 1 through MAX_POI, inclusive
        if index > MAX_POI or index == 0:
            return
        try:
            (self.x, self.y, self.z) = self.poi[index - 1]
        except ValueError as e:
            logging.debug(f"POI {index} has not been set")
            return
        logging.debug(f"Recalling to POI {index}: {self.x}, {self.y}, {self.z}")
        return self.send_cmd(f'G1 X{self.x} Y{self.y} Z{self.z}')
            
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
        except Exception as e:
            logging.error(f"MIDI open error: {e}")
            return None
    
    def __enter__(self):
        return self
    
    def clear_events(self):
        for msg in self.midi.iter_pending():
            pass

    def __exit__(self, exc_type, exc_value, traceback):
        # I think this one cleans up after itself?
        pass

def loop(args, jubilee, midi, light, piezo, schema):
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

    for msg in midi.midi:
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
            if control_node in str(msg):
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
                        light.send_cmd(f"A {int(angle)}")
                    elif name == "brightness-light":
                        bright = float(control_value) * (MAX_LIGHT / 127.0)
                        light.send_cmd(f"I {int(bright)}")
                    elif name == "nudge-piezo":
                        nudge = float(control_value) * (MAX_PIEZO / 127.0)
                        piezo.send_cmd(f"A {int(nudge)}")
                    else:
                        curvals[name] = control_value
                        if refvals[name] is None:
                            refvals[name] = control_value # first time through, grab the current value as the reference
                # buttons
                elif 'note=' in control_node:
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
                                jubilee.motors_off()
                            else:
                                logging.info("Motors are ON and origin is set")
                                jubilee.motors_on_set_zero()
                        elif name == 'ESTOP button':
                            for (name, val) in curvals.items():
                                refvals[name] = val
                            print("ESTOP hit, exiting")
                            jubilee.estop()
                            time.sleep(5)
                            return
                        elif 'set POI' in name:
                            maybe_poi = re.findall(r'^set POI (\d)', name)
                            if len(maybe_poi) == 1:
                                jubilee.set_poi(int(maybe_poi[0]))
                        elif 'recall POI' in name:
                            maybe_poi = re.findall(r'^recall POI (\d)', name)
                            if len(maybe_poi) == 1:
                                jubilee.recall_poi(int(maybe_poi[0]))
                        elif name == 'quit button':
                            jubilee.motors_off()
                            logging.info("Quitting controller...")
                            return
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
    args = parser.parse_args()
    numeric_level = getattr(logging, args.loglevel.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % args.loglevel)
    logging.basicConfig(level=numeric_level)
    # TODO: turn this into a command line argument
    iris_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1']

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
    jubilee = Jubilee(args.duet_port)
    midi = Midi(midimix_name)
    light = Light(iris_ports)
    piezo = Piezo(iris_ports)
    logging.info("MIDI-to-Jubilee Controller Starting. Motors are IDLE.")
    # wrap in 'with' so we can shut things down on exit if necessary
    with jubilee as j:
        with midi as m:
            with light as l:
                with piezo as p:
                    loop(args, j, m, l, p, schema)


if __name__ == "__main__":
    m = Thread(target=main, args=[])
    m.start()

    exit(cam())
