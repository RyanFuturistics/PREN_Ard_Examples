#!/usr/bin/python

import threading
import time
import logging
import math
import RPi.GPIO as GPIO




log = logging.getLogger(__name__)
GPIO.setmode(GPIO.BOARD)


class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin, n=3, offset=0):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.n = n
        self.counter = 0

        self.offset = offset

        self.values = [0] * n
        self.no_echo = False
        self.enabled = True

        log.debug("Setup Trigger %d, Echo %d" % (trigger_pin, echo_pin))
        GPIO.setup(echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(trigger_pin, GPIO.OUT)

        self.write_pin(0)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def read_pin(self):
        return GPIO.input(self.echo_pin)

    def write_pin(self, state):
        GPIO.output(self.trigger_pin, state)

    def trigger(self):
        self.write_pin(1)
        time.sleep(.00001)
        self.write_pin(0)

    def convert_to_cm(self, delay_time):
        return (delay_time * 1E6) / 58

    def mean(self):
        return float(sum(self.values)) / max(len(self.values), 1)

    def get_last_value(self):
        return self.values[self.n - 1]

    def get_former_value(self):
        return self.values[self.n - 2]

    def __get_value__(self):
        log.debug("Trigger on pin: %d" % self.trigger_pin)
        self.trigger()

        start = time.time()
        c2 = GPIO.wait_for_edge(self.echo_pin, GPIO.RISING, timeout=60)
        pulse = time.time()
        c2 = GPIO.wait_for_edge(self.echo_pin, GPIO.FALLING, timeout=30)

        pulse_duration = time.time() - pulse

        if c2 is None:
            log.debug("No Echo on pin: %d " % self.echo_pin)
            self.no_echo = True


        else:
            self.no_echo = True
            distance = self.convert_to_cm(pulse_duration) - self.offset
            log.info("echo " + str(distance) + "cm from pin:" + str(self.trigger_pin))
            measurement_time = time.time() - start

            self.counter += 1

            self.values[self.counter % self.n] = distance

            if measurement_time < .06:
                log.debug("delay for next measurement")
                time.sleep(.06 - measurement_time)
            return distance


class SensorArray:

    LF = 5
    LR = 4
    RF = 1
    RR = 0
    FL = 3
    FR = 2

    offsets = [0, 0, 0, 0, 0, 0]

    trigger_pins = [3, 5, 7, 11, 13, 19] # 15
    echo_pins = [16, 18, 22, 24, 26, 40]# 28 doesn't work for some reason

    def __init__(self, n_avg=3):
        self.sensors = [None] * len(self.trigger_pins)
        self.n_sensors = range(len(self.trigger_pins))
        self.running = True
        self.n_avg=n_avg

        for sensor_nr in self.n_sensors:
            self.__add_sensor__(sensor_nr)



    def __add_sensor__(self, nr):
        self.sensors[nr] = UltrasonicSensor(self.trigger_pins[nr], self.echo_pins[nr], n = self.n_avg, offset=self.offsets[nr])

    def run(self):
        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def loop(self):
        self.running = True
        while self.running:
            for sensor in self.sensors:
                if sensor.enabled:
                    sensor.__get_value__()

    def disable_all(self):
        for sensor in self.sensors:
               sensor.disable()

    def enable_all(self):
        for sensor in self.sensors:
               sensor.enable()

    def angle(self, s1, s2, s_distance):

        f = self.sensors[s1].mean()
        r = self.sensors[s2].mean()

        delta = f - r
        return math.atan2(delta, s_distance)


        # 126mm sensor distance
    def left_angle(self):
        return self.angle(self.LF, self.LR, 12.6)

    def right_angle(self):
        return self.angle(self.RF, self.RR, 12.6)

        # 109mmm sensor distance
    def front_angle(self):
        return self.angle(self.FL, self.FR, 10.9)

    def distance(self, s1, s2):
        d1 = self.sensors[s1].mean()
        d2 = self.sensors[s2].mean()

        return (d1+d2) / 2

    def left_distance(self):
        return self.distance(self.LF, self.LR)

    def right_distance(self):
        return self.distance(self.RF, self.RR)

    def front_distance(self):
        return self.distance(self.FL, self.FR)

    def LF_distance(self):
        return self.sensors[self.LF].mean()

    def RF_distance(self):
        return self.sensors[self.RF].mean()

    def enable_right_front(self):
        self.sensors[self.RF].enable()

    def enable_left_front(self):
        self.sensors[self.LF].enable()

    def enable_right(self):

        self.sensors[self.RF].enable()
        self.sensors[self.RR].enable()

    def enable_left(self):

        self.sensors[self.LF].enable()
        self.sensors[self.LR].enable()

    def enable_front(self):

        self.sensors[self.FL].enable()
        self.sensors[self.FR].enable()

    def stop_measurement_loop(self):
        self.running = False


if __name__ == "__main__":
    # logs sensor data on disk

    a = SensorArray()

    #t1 = threading.Thread(target=a.start_partial_loop, args=([SensorArray.LF, SensorArray.LR],))
    a.run()


    time.sleep(1)

    while True:
        with open("sensor.log", "a") as f:
            for i, sensor in enumerate(a.sensors):
                f.write("{:.2f};".format(sensor.mean()))

                print i, "%.2f" % sensor.mean(),
            print "*"

            f.write("\n")
            time.sleep(.06 * len(a.sensors))
            #time.sleep(.12) #########################################################################DEBUG

    a.stop_measurement_loop()
    t1.join()
    log.info("stoped gracefully")
