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

SCHEMA_STORAGE = 'miduet-config.json'
SCHEMA_VERSION = 2
MAX_POI = 3
MIN_ANGLE = -100
MAX_ANGLE = 100
MAX_LIGHT = 4096
MAX_PIEZO = 8192 #16383
MAX_GAMMA = 2.0

cam_quit = Event()

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
            self.poi = [None] * MAX_POI # this is an array of Poi objects
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

        # NOTE AXIS SWAP - this is more intuitive based on the orientation
        # of the machine relative to my seat.
        # FIXME you must also change the axis order in recall_poi() -- this should probably be...made less brittle...
        if 'x' in axis:
            self.x += step
            self.send_cmd(f'G1 Y{self.x}')
        elif 'y' in axis:
            self.y += step
            self.send_cmd(f'G1 X{self.y}')
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
        self.send_cmd(f'G1 Y{self.x} X{self.y} Z{self.z}')
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

def quitter(jubilee, light, piezo, cam_quit):
    cam_quit.wait()
    jubilee.motors_off()
    light.send_cmd("I 0")
    piezo.send_cmd("A 0")
    logging.info("Safe quit reached!")

def all_leds_off(schema, midi):
    # turn off all controller LEDs
    for control_node in schema.values():
        if type(control_node) is str:
            if 'note=' in control_node:
                note_id = int(control_node.split('=')[1])
                midi.set_led_state(note_id, False)

def loop(args, jubilee, midi, light, piezo, gamma, schema):
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
                        light.send_cmd(f"A {int(angle)}")
                        last_angle = int(angle)
                    elif name == "brightness-light":
                        bright = float(control_value) * (MAX_LIGHT / 127.0)
                        light.send_cmd(f"I {int(bright)}")
                        last_intensity = int(bright)
                    elif name == "nudge-piezo":
                        nudge = float(control_value) * (MAX_PIEZO / 127.0)
                        piezo.send_cmd(f"A {int(nudge)}")
                        last_piezo = int(nudge)
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
                                jubilee.set_poi(int(maybe_poi[0]), last_angle, last_intensity, last_piezo)
                        elif 'recall POI' in name:
                            maybe_poi = re.findall(r'^recall POI (\d)', name)
                            if len(maybe_poi) == 1:
                                maybe_poi = jubilee.recall_poi(int(maybe_poi[0]))
                                if maybe_poi is not None:
                                    (l_t, l_i, l_p) = maybe_poi
                                    logging.debug(f"Recalling poi {l_t} {l_i} {l_p}")
                                    if l_t is not None:
                                        light.send_cmd(f"A {l_t}")
                                    if l_i is not None:
                                        light.send_cmd(f"I {l_i}")
                                    if l_p is not None:
                                        piezo.send_cmd(f"A {l_p}")
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
    args = parser.parse_args()

    gamma = Gamma()
    if not args.no_cam:
        c = Thread(target=cam, args=[cam_quit, gamma])
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
                        args, j, m, l, p, gamma, schema
                    ])
                    l.start()
                    # when the quitter exits, everything has been brought down in an orderly fashion
                    q.join()
                    logging.debug("Midi control thread reached quit")

if __name__ == "__main__":
    main()

