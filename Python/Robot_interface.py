import serial
import logging

logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger(__name__)


class Robot:

    def __init__(self, port="/dev/ttyUSB0"):
        self.handle = serial.Serial(port=port, baudrate=115200, timeout=1)
        self.speed = 1
        self.left_ticks = 0
        self.right_ticks = 0
        self.cm_to_ticks = 42
        state = "ok"

    def send_cmd(self, cmd):
        try:
            self.handle.write(cmd + "\n")
        except Exception as e:
            self.state = "Error2"
            log.critical("Serial Write Error")

    def read(self):
        data =  self.handle.readline()[:-1]
        try:
            self.state, self.left_ticks, self.right_ticks = data.split(",")
        except Exception as e:
            self.state = "Error2"
            log.critical("Serial Write Error")

    def set_speed(self, speed):
        if 0 < speed < 256:
            self.speed = speed
            log.debug("Set speed value to " + str(speed))
        else:
            log.critical("Received invalid speed value: " + str(speed))

    def move_fwd(self, cm):
        self.move_fwd_ticks(cm * self.cm_to_ticks)

    def move_rev(self, cm):
        self.move_rev_ticks(cm * self.cm_to_ticks)

    def move_fwd_ticks(self, ticks):
        log.debug("FWD " + str(ticks) + "ticks")
        self.send_cmd("C{:010d}{:03d}1".format(ticks, self.speed))

    def move_rev_ticks(self, ticks):
        log.debug("FWD " + str(ticks) + "ticks")
        self.send_cmd("C{:010d}{:03d}0".format(ticks, self.speed))

    def fwd(self, left_speed, right_speed):
        log.debug("FWD L, R Speed " + str(left_speed) + "," + + str(right_speed) )
        self.send_cmd("M{:03d}{:03d}0".format(left_speed, right_speed))

    def fwd(self, left_speed, right_speed):
        log.debug("REV L, R Speed " + str(left_speed) + "," + + str(right_speed) )
        self.send_cmd("M{:03d}{:03d}0".format(left_speed, right_speed))

    def left(self, degrees):
        log.debug("turn left" + str(degrees))
        self.send_cmd("L{:010d}0".format(degrees))

    def right(self, degrees):
        log.debug("turn right" + str(degrees))
        self.send_cmd("R{:010d}0".format(degrees))

    def stop(self):
        log.debug("STOP ")
        self.send_cmd("!")