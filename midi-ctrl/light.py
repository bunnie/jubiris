import logging
import iqmotion as iq
import serial
import time
import math

MAX_LIGHT = 4096

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
