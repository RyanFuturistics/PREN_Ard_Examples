#!/usr/bin/python

import threading
import time, random
import logging
import RPi.GPIO as GPIO
import math

logging.basicConfig(level=logging.CRITICAL)

log = logging.getLogger(__name__)
GPIO.setmode(GPIO.BOARD)


class Ultrasonic_Sensor:

    def __init__(self, trigger_pin, echo_pin, n=10):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.n = n
        self.counter=0

        self.values = [0] * n
        self.no_echo = False

        GPIO.setup(echo_pin, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
        GPIO.setup(trigger_pin, GPIO.OUT)

        self.write_pin(0)

    def read_pin(self):
        return GPIO.input(self.echo_pin)

    def write_pin(self, state):
        GPIO.output(self.trigger_pin, state)

    def trigger(self):
        self.write_pin(1)
        time.sleep(.00001)
        self.write_pin(0)

    def convert_to_cm(self, delay_time):
        return (delay_time * 1E6 ) / 58

    def mean(self):
        return float(sum(self.values)) / max(len(self.values), 1)

    def get_value(self):
        log.debug("Trigger")
        self.trigger()

        start = time.time()
        c = GPIO.wait_for_edge(self.echo_pin, GPIO.RISING, timeout=60)
        pulse = time.time()
        c2 = GPIO.wait_for_edge(self.echo_pin, GPIO.FALLING, timeout=30)

        pulse_duration = time.time()-pulse

        if c2 is None:
             log.debug("No Echo")
             self.no_echo = True

        else:
            self.no_echo = True
            distance = self.convert_to_cm(pulse_duration)
            log.info( "echo " + str(distance) + "cm")
            measurement_time = time.time() - start

            self.counter += 1

            self.values[self.counter % self.n] = distance

            if measurement_time < .06:
                log.debug("delay for next measurement")
                time.sleep(.06 - measurement_time)
            return distance

class Sensor_Array:
    def __init__(self):
        self.sensors = []
        self.running = True;

    def addSensor(self, Sensor):
        self.sensors.append(Sensor)

    def start_measurement_loop(self):
        self.running = True
        while self.running:
            for sensor in self.sensors:
                sensor.get_value()

    def stop_measurement_loop(self):
        self.running = False



if __name__ == "__main__":

    s1 = Ultrasonic_Sensor(3, 11)

    a = Sensor_Array()
    a.addSensor(s1)

    t1 = threading.Thread(target= a.start_measurement_loop)
    t1.start()
    time.sleep(1)

    for i in range(10):
        print "***************", s1.mean()
        time.sleep(.7)

    a.stop_measurement_loop()
    t1.join()
    log.info("stoped gracefully")

