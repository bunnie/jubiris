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

def loop(args, ser, midi_in):
    ser.write(b'G91\nG1 X1\n')
    while True:
        data = ser.readline()
        if data:
            print(data)
            if b'ok\n' in data:
                break

def main():
    parser = argparse.ArgumentParser(description="MIDI-to-Duet controller")
    parser.add_argument(
        "--duet-port", required=False, help="path to duet3d serial port", default='/dev/ttyACM0'
    )
    parser.add_argument(
        "--midi-port", required=False, help="MIDI device name to use"
    )
    args = parser.parse_args()

    if args.midi_port is not None:
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

    try:
        ser = serial.Serial(args.duet_port, 115200, timeout=1)
        print("Duet3D serial port opened")

        with mido.open_input(midimix_name) as midi_in:
            loop(args, ser, midi_in)

    except serial.SerialException as e:
        print(f"Serial error: {e}")

    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
    exit(0)
