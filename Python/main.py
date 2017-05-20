import Robot_interface
from  UltrasonicSensor import SensorArray
from  TcpConnect import TcpClient
import Hardware_Interface.Switch
import Hardware_Interface.Display

import threading, logging, time

logging.basicConfig(level=logging.DEBUG)

log = logging.getLogger(__name__)

go = False
def update():
    robot.read()
    if vision_handle.data_available():
        while len(vision_handle.queue) > 1:
            data = vision_handle.queue.pop(0)
            if data == "GO\n":
                go = True
            elif data[:6]=="number":
                nr = int(data[6:7])
                display.display_number(nr)
            elif data == "STOP\n":
                go = False



vision_handle = TcpClient.TcpClient()
robot = Robot_interface.Robot()
course_selector = Hardware_Interface.Switch(33)
display = Hardware_Interface.Display([35, 37, 38, 36])
switch = Hardware_Interface.Switch(31)

sensors = SensorArray()
sensors.add_Sensors()



t1 = threading.Thread(target= sensors.start_measurement_loop)
t1.start()
time.sleep(1)
log.info("Sensor Array Initalised")

while not go:
    update()
    pass

if course_selector.left_course():
    log.info("Left Course selected")
else:
    log.info("Right Course selected")



robot.fwd(128,128)

while sensors[SensorArray.LF].mean() < 100:
    update()

robot.move_fwd(20)

if course_selector.left_course():
    robot.left(90)
else:
    robot.right(90)



