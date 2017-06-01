import serial, time
import logging


log = logging.getLogger(__name__)


class Robot:

    Stop = 0
    FWD = 1
    REV = 2
    LEFT = 4
    RIGHT = 3
    OK = 0
    BUSY = 1
    ERROR = 2


    # 0 Stop, 1 FW FW, 2 L_RV, R_RV, 3 L_FW RWV, 4 L_RV, R_FW

    def __init__(self, port="/dev/ttyACM0"):
        self.handle = serial.Serial(port=port, baudrate=38400, timeout=.5)
        self.speed = 1
        self.left_ticks = 0
        self.right_ticks = 0
        self.cm_to_ticks = 42
        self.state = 0
        self.error = 0

    def send_cmd(self, cmd):
        try:
            log.debug("send to robot <" + cmd + ">\n")
            self.handle.write(cmd + "\n")
        except Exception as e:
            self.state = "Error2"
            log.critical("Serial Write Error")

        self.read_reponse()



    def get_status(self):
       self.send_cmd("I")

    def read_reponse(self):

        if not self.handle.inWaiting():
            time.sleep(.02)

        while self.handle.inWaiting():
            data =  self.handle.readline()[:-1]

            try:
                items =  data.split(",")
                self.state = int(items[0])

                log.info("robot response: <%s>:" % data)

                if self.state == self.OK or self.state == self.BUSY or self.state == self.ERROR:
                    self.error, self.left_ticks, self.right_ticks = int(items[1]), int(items[2]), int(items[3])
                else:
                    log.critical("Arduino Serial Communication Problem Received string <%s>:" % data)
            except Exception as e:

                self.state = "Error2"
                log.critical("Serial Write Error")
                log.critical(e.message)

    def set_speed(self, speed):
        if 0 <= speed <= 1600:
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
        self.send_cmd("C{:010d},{:03d},{:1d}".format(ticks, self.speed, self.FWD))

    def move_rev_ticks(self, ticks):
        log.debug("FWD " + str(ticks) + "ticks")
        self.send_cmd("C{:010d},{:03d},{:1d}".format(ticks, self.speed, self.REV))

    def fwd(self, left_speed, right_speed):
        log.debug("FWD L, R Speed " + str(left_speed) + "," + str(right_speed) )
        self.send_cmd("M{:03d},{:03d},{:1d}".format(left_speed, right_speed, self.FWD))

    def rev(self, left_speed, right_speed):
        log.debug("REV L, R Speed " + str(left_speed) + "," + str(right_speed) )
        self.send_cmd("M{:03d},{:03d},{:1d}".format(left_speed, right_speed, self.REV))

    def left(self, left_speed, right_speed):
        log.debug("FWD L, R Speed " + str(left_speed) + "," + str(right_speed) )
        self.send_cmd("M{:03d},{:03d},{:1d}".format(left_speed, right_speed, self.LEFT))

    def right(self, left_speed, right_speed):
        log.debug("REV L, R Speed " + str(left_speed) + "," + str(right_speed) )
        self.send_cmd("M{:03d},{:03d},{:1d}".format(left_speed, right_speed, self.RIGHT))

    def left_turn(self, degrees):
        log.debug("turn left" + str(degrees))
        self.send_cmd("L{:03d}".format(degrees))

    def right_turn(self, degrees):
        log.debug("turn right" + str(degrees))
        self.send_cmd("R{:03d}".format(degrees))

    def stop(self):
        log.debug("STOP ")
        self.send_cmd("S")
        #self.send_cmd("M0,0,0")

if __name__ == "__main__":
    pass


