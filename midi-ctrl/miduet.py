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

SCHEMA_STORAGE = 'miduet-config.json'
SCHEMA_VERSION = 2

def ser_cmd(ser, cmd):
    cmd_bytes = bytes(cmd + '\n', 'utf-8')
    print(str(cmd_bytes, 'utf-8'))
    ser.write(cmd_bytes)
    line = ''
    timeout = True
    while True:
        c = ser.read(1)
        if len(c) == 0:
            break
        line += c.decode('utf-8')
        if 'ok\n' in line:
            timeout = False
            break

    return timeout

def step_axis(ser, name, step):
    if step == 0.0:
        return
    # "home" the machine
    ser_cmd(ser, 'G91')
    if 'x' in name:
        ser_cmd(ser, f'G1 X{step}')
    elif 'y' in name:
        ser_cmd(ser, f'G1 Y{step}')
    elif 'z' in name:
        ser_cmd(ser, f'G1 Z{step}')
    else:
        print('axis not yet implemented')

def loop(args, ser, midi_in, schema):
    # clear any stray events in the queue
    for msg in midi_in.iter_pending():
        pass

    # set the current position as 0
    ser_cmd(ser, 'G92 X0 Y0 Z0 U0')
    ser_cmd(ser, 'M18') # disable all stepper motors
    motor_state = 'IDLE'

    refvals = {}
    curvals = {}
    for name in schema.keys():
        refvals[name] = None
        curvals[name] = None

    for msg in midi_in:
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
                # slider case
                if 'control=' in control_node:
                    # extract the value
                    for i in str(msg).strip().split(' '):
                        if 'value=' in i:
                            control_value = i.split('=')[1]
                            break
                    assert(control_value is not None)
                    curvals[name] = control_value
                    valid = True
                    if refvals[name] is None:
                        refvals[name] = control_value # first time through, grab the current value as the reference

                elif 'note=' in control_node:
                    if control_node in str(msg):
                        if 'note_on' in str(msg):
                            # the button was hit, that's all we care about
                            if name == 'zero button':
                                valid = True
                                print("zero")
                                for (name, val) in curvals.items():
                                    refvals[name] = val
                            elif name == 'ESTOP button':
                                for (name, val) in curvals.items():
                                    refvals[name] = val
                                print("ESTOP hit, exiting")
                                ser_cmd(ser, 'M112')
                                time.sleep(5)
                                exit(0)
        
        # now update machine position based on values
        if valid:
            for (name, val) in curvals.items():
                if refvals[name] is not None:
                    delta = int(curvals[name]) - int(refvals[name])
                    refvals[name] = curvals[name] # reset ref
                    if delta > 0:
                        if 'coarse' in name:
                            step = 1.0
                        else:
                            step = 0.05
                    elif delta < 0:
                        if 'coarse' in name:
                            step = -1.0
                        else:
                            step = -0.05
                    else:
                        step = 0.0
                    if 'z' in name:
                        step = step / 5.0
                    print(f"Delta of {delta} for {name}")
                    if step_axis(ser, name, step):
                        print("Motor command timeout!")
                        # clear any stray events in the queue
                        for msg in midi_in.iter_pending():
                            pass


    # ser.write(b'G91\nG1 X1\n')

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
        'zero-x button' : None,
        'zero-y button' : None,
        'zero-z button' : None,
        'idle button': None,
        'set POI 1 button' : None,
        'set POI 2 button' : None,
        'set POI 3 button' : None,
        'recall POI 1 button' : None,
        'recall POI 2 button' : None,
        'recall POI 3 button' : None,
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
    args = parser.parse_args()

    if args.midi_port is None:
        input_list = mido.get_input_names()
        midimix_name = None
        for i in input_list:
            if 'MIDI Mix' in i:
                midimix_name = i
                break
        if midimix_name is None:
            print("couldn't find Akai MIDI Mix device!")
            return
    else:
        midimix_name = args.midi_port

    if args.set_controls:
        try:
            with mido.open_input(midimix_name) as midi_in:
                set_controls(midi_in)
        except e:
            print("Couldn't open MIDI device, --set-controls has failed.")
            print(e)
            exit(1)

        # after setting controls, exit
        print("Control setting finished. Please restart without --set-controls.")
        exit(0)
    else:
        try:
            with open(SCHEMA_STORAGE, "r") as config:
                schema = json.loads(config.read())
        except:
            print("Couldn't load control configuration. Try running --set-controls to setup controls.")
            exit(1)

    try:
        ser = serial.Serial(args.duet_port, 115200, timeout=1.0)
        print("Duet3D serial port opened")

        with mido.open_input(midimix_name) as midi_in:
            loop(args, ser, midi_in, schema)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
    exit(0)
