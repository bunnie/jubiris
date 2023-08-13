#!/usr/bin/env python3

# packages installed:
#  - sudo apt-get install python3-pip
#  - sudo apt-get install alsa-tools alsa-utils libasound2-dev
#  - python3 -m pip install mido
#  - python3 -m pip install python-rtmidi  # note, NOT the 'rtmidi' package, it is incompatible
#  - sudo ln -s /usr/lib/x86_64-linux-gnu/alsa-lib/ /usr/lib64/alsa-lib  # fixes a library path issue that might just be a problem with ubuntu 22.04
#
# Footguns in installation:
#  - there are 3 packages that would seem to be 'rtmidi'. Only the one named 'python-rtmidi' works.
#  - There is a packaging problem in ALSA shared libraries and 'multiarch' support. This might be unique to Ubuntu 22.04. But, ALSA wants to find its
#    shared libraries in /usr/lib64/alsa-lib. However, they are actually installed to /usr/lib/x86_64-linux-gnu/alsa-lib. A symlink
#    fixes that issue.

import mido

input_list = mido.get_input_names()
midimix_name = None
for i in input_list:
    if 'MIDI Mix' in i:
        midimix_name = i
        break

if midimix_name is None:
    print("couldn't find Akai MIDI Mix device!")
    exit(1)

with mido.open_input(midimix_name) as inport:
    with mido.open_output(midimix_name) as outport:
        for msg in inport:
            print(msg)
            elems = str(msg).split(' ')
            for e in elems:
                if 'note=' in e:
                    n = int(e.split('=')[1])
                    m = mido.Message('note_on', channel=0, note=n, velocity=0)
                    outport.send(m)

