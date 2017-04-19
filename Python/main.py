import Robot_interface
from  Ultrasonic_Sensor import Sensor_Array, Ultrasonic_Sensor
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
            elif data[:7]=="number":
                nr = int(data[6:])
                display.display_number(nr)



vision_handle = TcpClient.TcpClient()
robot = Robot_interface.Robot()
course_selector = Hardware_Interface.Switch(PIN)
display = Hardware_Interface.Display(PIN)

sensors = Sensor_Array()
sensor_left1 = Ultrasonic_Sensor()
sensors.addSensor(sensor_left1)

sensor_left2 = Ultrasonic_Sensor()
sensors.addSensor(sensor_left2)

sensor_front1 = Ultrasonic_Sensor()
sensors.addSensor(sensor_front1)

sensor_front2 = Ultrasonic_Sensor()
sensors.addSensor(sensor_front2)

sensor_right1 = Ultrasonic_Sensor()
sensors.addSensor(sensor_right1)

sensor_right2 = Ultrasonic_Sensor()
sensors.addSensor(sensor_right2)

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

while sensor_left1.mean() < 100:
    update()

robot.move_fwd(20)

if course_selector.left_course():
    robot.left(90)
else:
    robot.right(90)



