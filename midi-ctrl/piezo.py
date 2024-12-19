import logging
import serial

PIEZO_UM_PER_VOLT = 1.17 # estimated
PIEZO_MAX_VOLT = 103.97
PIEZO_MAX_CODE = 16383 # max scale overall
PIEZO_UM_PER_LSB = 0.007425 # (1 / 193.5) # 0.007425
MAX_PIEZO = 16383 # max scale for slider (set to 8192 for 20x objective)

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

    def is_code_valid(self, a):
        if a < 0:
            return False
        if a > PIEZO_MAX_CODE:
            return False
        return True

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
