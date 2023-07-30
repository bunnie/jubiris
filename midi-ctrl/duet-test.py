#!/usr/bin/env python3

# python3 -m pip install pyserial

import serial

serial_port = '/dev/ttyACM0'

try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print("serial port opened")

    ser.write(b'G91\nG1 X1\n')
    while True:
        data = ser.readline()
        if data:
            print(data)
            if b'ok\n' in data:
                break

except serial.SerialException as e:
    print(f"Serial error: {e}")

finally:
    if ser.is_open:
        ser.close()
        print("serial port closed")

# import requests
# JUBILEE_URL = "http://10.0.245.6"
# requests.get(JUBILEE_URL + "/rr_connect?password=")
# response = requests.get(JUBILEE_URL + "/rr_download?name=/sys/config.g")
# print(response.text)

