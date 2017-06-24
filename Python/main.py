#!/usr/bin/python
import Robot_interface
from  UltrasonicSensor import SensorArray
from  TcpConnect import TcpClient
from Hardware_Interface import Switch
from Hardware_Interface import Display

import threading, logging, time

logging.basicConfig(level=logging.CRITICAL)

#logging.basicConfig(filename='error.log',level=logging.INFO)


log = logging.getLogger(__name__)

# go = False
# def update():
#     robot.read()
#     if vision_handle.data_available():
#
#             if data == "GO\n":
#                 pass
#                 #go = True
#             elif data[:7]=="number":
#                 nr = int(data[6:])
#                 display.display_number(nr)



#vision_handle = TcpClient.TcpClient("192.168.1.1", 4444)
robot = Robot_interface.Robot()
course_selector = Switch(33)
switch = Switch(31)

sensors = SensorArray()
sensors.run()

#robot.set_pid(.28, 1.00,.0004)
robot.set_pid(.49, 0, .0004)

sensors.disable_all()
sensors.enable_right_front()
#sensors.enable_left_front()

robot.stop()

class Statemachine:
    speed = 200

    Kp = 1.5
    Ki = 0
    Kd = 0
    last_ticks = 0


    def waiting_for_go_state(self):
        if vision_handle.data_available():
            if vision_handle.queue.pop(0) == "Go\n":
                log.critical("Go received")
                sensors.disable_all()
                sensors.enable_right_front()
                sensors.enable_left_front()
                return self.test_state

        return self.waiting_for_go_state

    def test_state(self):
        vision_handle.sendToServer("Number\n")
        if vision_handle.data_available():
            data = vision_handle.queue.pop(0)[:-1]
            print data
            if data[0] == "N":
                nr = data[1]
                log.critical("Number %d" % int(nr))
                return self.go_state
        else:
            time.sleep(1)
        return self.test_state

    def go_state(self):
        robot.fwd(80, 80)



        if sensors.LF_distance() + sensors.RF_distance() < 30:
            log.critical("Straight1 State")
            return self.straight1_state

        else:
            return self.go_state

    def c_test(self):
        robot.fwd(100,100)
        start = time.time()
        while time.time() - start < 2:
            robot.read_pid_out()

        robot.rev(100,100)

        start = time.time()
        while time.time() - start < 2:
            robot.read_pid_out()

        return self.c_test


    def straight1_state(self):

        if  sensors.sensors[sensors.RF].mean() < 500:

            d = sensors.RF_distance()
            d = sensors.sensors[sensors.RF].mean()
            delta = 20-d


            setR = int(self.speed + self.Kp * delta)
            setL = int(self.speed - self.Kp * delta)

            if delta > .5:
                robot.fwd(80, 120)

                #
                # while robot.state != robot.OK:
                #     robot.get_status()
                #     robot.fwd(self.speed, self.speed)

                #time.sleep(.2)
                #robot.fwd(50, self.speed)
            elif delta < -.5:
                robot.fwd(1200, 1200)
            else:
                robot.fwd(self.speed, self.speed)
                #time.sleep(.2)
                #robot.fwd(self.speed, self.speed / 2)



            print delta, setL, setR

            return self.straight1_state
        else:

            log.critical("endStraight State")
            robot.get_status()
            last_ticks = robot.left_ticks
            return self.straight1_state()

    def end_straight_state(self):


        robot.stop()
        robot.move_fwd(70)
        robot.get_status()

        while robot.state != robot.OK:
            robot.get_status()

        if sensors.sensors[sensors.RF].mean() < 20:
            return self.turn_state
        else:
            return self.straight1_state


    def turn_state(self):
        robot.right_turn(90)
        robot.get_status()
        if robot.state == robot.OK:
            robot.get_status()
            self.last_ticks = robot.left_ticks
            log.critical("Turn FWD State")
            return self.turn_fwd_state
        return self.turn_fwd_state

    def turn_fwd_state(self):
        robot.stop()
        robot.move_fwd(70)
        robot.get_status()

        while robot.state != robot.OK:
            robot.get_status()


    def turn2_state(self):
        robot.right_turn(90)
        robot.get_status()
        if robot.state == robot.OK:
            return self.bottleneck_state

    def bottleneck_state(self):
        pass




statemachine = Statemachine()

state = statemachine.c_test

time.sleep(1)
while state:
    state = state()




# data = vision_handle.queue.pop(0)


