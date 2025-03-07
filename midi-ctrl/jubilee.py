import logging
import serial
import time

MAX_POI = 3
MIN_ROTATION_ANGLE = -100.0
MAX_ROTATION_ANGLE = 100.0
CONTROL_MIN_ROTATION_ANGLE = -80.0 # somewhat not exactly degrees somehow...
CONTROL_MAX_ROTATION_ANGLE = 80.0
MIN_JUBILEE_STEP_MM = 0.03 # must be big enough to encompass a focus sweep!

SAFETY_XY = 40.0
SAFETY_ZMIN = 8.0
SAFETY_ZMAX = 15.0

class Poi():
    def __init__(self, x, y, z, piezo):
        self.x = x
        self.y = y
        self.z = z
        self.piezo = piezo

    def __json_default__(self):
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'piezo': self.piezo
        }

class Jubilee:
    def __init__(self, port):
        self.port = port
        self.state = 'OFF'
        self.x = 0.0
        self.y = 0.0
        self.z = 10.0
        self.r = 0.0
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

        if 'x' in axis and abs(self.x) > SAFETY_XY:
            logging.warning(f"X would step out of range: {self.x}")
            self.x -= step # undo the step
            return False
        elif 'y' in axis and abs(self.y) > SAFETY_XY:
            logging.warning(f"Y would step out of range: {self.y}")
            self.y -= step
            return False
        elif 'z' in axis and (self.z < SAFETY_ZMIN or self.z > SAFETY_ZMAX): # bind this tightly to a zone around 10.0, which is the default starting value
            logging.warning(f"Z would step out of range: {self.z}")
            self.z -= step
            logging.warning("Requested Z on set_axis() is outside of a conservatively bound range.\nIf the machine is definitely safe, re-zero the motors and try again.")
            return False

        return self.sync_to_mach()

    # sets one axis to a value. Only has an effect if the value changes
    # the machine state versus the current recorded state. This is because
    # re-writing the same value to the machine can still cause the microstepping
    # to fluctuate and affect focus/vibrations.
    def set_axis(self, axis, value):
        updated = False
        if not self.is_on():
            return False # don't return an error, just silently fail
        if (axis == 'x' or axis == 'y') and abs(value) > SAFETY_XY: # outside a reasonable request
            return False
        if axis == 'z' and (value < SAFETY_ZMIN or value > SAFETY_ZMAX): # bind this tightly to a zone around 10.0, which is the default starting value
            logging.warning("Requested Z on set_axis() is outside of a conservatively bound range.\nIf the machine is definitely safe, re-zero the motors and try again.")
            return False

        if 'x' in axis:
            if value != self.x:
                self.x = value
                updated = True
        elif 'y' in axis:
            if value != self.y:
                self.y = value
                updated = True
        elif 'z' in axis:
            if value != self.z:
                self.z = value
                updated = True
        else:
            print('axis not yet implemented')

        if updated:
            return self.sync_to_mach()
        else: # if the value is a no-op, don't send the command.
            return True

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
        return self.send_cmd(f'G1 Y{-self.x:0.2f} X{-self.y:0.2f} Z{self.z:0.2f}')

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
        self.send_cmd(f'M208 X{-SAFETY_XY}:{SAFETY_XY} Y{-SAFETY_XY}:{SAFETY_XY} Z{SAFETY_ZMIN}:{SAFETY_ZMAX}') # limits on travel
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
