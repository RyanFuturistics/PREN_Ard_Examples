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



vision_handle = TcpClient.TcpClient(TcpClient, "192.168.1.1", 4444)
robot = Robot_interface.Robot()
course_selector = Switch(33)
switch = Switch(31)

sensors = SensorArray()
sensors.run()



class Statemachine:
    speed = 120

    Kp = .5
    Ki = 0
    Kd = 0
    last_ticks = 0


    def waiting_for_go_state(self):
        if vision_handle.data_available():
            if vision_handle.queue.pop(0) == "GO\n":
                log.critical("Go received")
                sensors.disable_all()
                sensors.enable_right_front()
                sensors.enable_left_front()
                return self.go_state

        return self.waiting_for_go_state

    def go_state(self):
        robot.fwd(80, 80)
        if sensors.LF_distance() + sensors.RF_distance() < 30:
            log.critical("Straight1 State")
            return self.straight1_state

        else:
            return self.go_state


    def straight1_state(self):

        dl, dr = sensors.LF_distance(), sensors.RF_distance()
        if (dl + dr) < 35:

            dl, dr = sensors.sensors[sensors.LF].mean(), sensors.sensors[sensors.RF].mean()
            delta = abs(dl - dr)

            if dl > dr:
                robot.fwd(int(self.speed - self.Kp * delta), int(self.speed + self.Kp * delta))
            else:
                robot.fwd(int(self.speed + self.Kp * delta), int(self.speed - self.Kp * delta))

            print dl, dr, int(self.speed - self.Kp * delta), int(self.speed + self.Kp * delta)

            return self.straight1_state
        else:

            log.critical("endStraight State")
            robot.get_status()
            last_ticks = robot.left_ticks
            return self.end_straight_state

    def end_straight_state(self):
        robot.get_status()
        current_ticks = robot.left_ticks

        dl, dr = sensors.LF_distance(), sensors.RF_distance()
        if (dl + dr) < 30:
            return self.straight1_state

        elif current_ticks - self.last_ticks > 1000:
            robot.stop()
            log.critical("Turn State")
            return self.turn_state
        return self.end_straight_state

    def turn_state(self):
        robot.left_turn(90)
        robot.get_status()
        if robot.state == robot.OK:
            robot.get_status()
            self.last_ticks = robot.left_ticks
            log.critical("Turn FWD State")
            return self.turn_fwd_state
        return self.turn_fwd_state

    def turn_fwd_state(self):
        robot.get_status()
        current_ticks = robot.left_ticks
        if current_ticks - self.last_ticks > 1000:
            robot.stop()
            return self.turn2_state()

        return self.turn_fwd_state


    def turn2_state(self):
        robot.left_turn(90)
        robot.get_status()
        if robot.state == robot.OK:
            return self.bottleneck_state

    def bottleneck_state(self):



statemachine = Statemachine()

state = statemachine.waiting_for_go_state


while state: state = state()




# data = vision_handle.queue.pop(0)


time.sleep(1)




print "Passage"





print "TURN"

robot.get_status()
time.sleep(.5)
while  (dl + dr) < 35:
    pass

robot.fwd(speed,speed)
time.sleep(7)

if course_selector:
    robot.right_turn(90)
    robot.get_status()
    while robot.state != robot.OK:
        robot.get_status()
    robot.fwd(speed, speed)
    time.sleep(7)
    robot.right_turn(90)
    robot.get_status()
    while robot.state != robot.OK:
        robot.get_status()

else:
    robot.left_turn(90)
    robot.get_status()
    while robot.state != robot.OK:
        robot.get_status()
    robot.fwd(speed, speed)
    time.sleep(7)
    robot.left_turn(90)
    robot.get_status()
    while robot.state != robot.OK:
        robot.get_status()

robot.stop()




print "stop"

print"stop"
#robot.stop()



