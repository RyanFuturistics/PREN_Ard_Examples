#!/usr/bin/python

import threading
import time
import logging
import RPi.GPIO as GPIO


logging.basicConfig(level=logging.CRITICAL)

log = logging.getLogger(__name__)
GPIO.setmode(GPIO.BOARD)


class UltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin, n=10):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.n = n
        self.counter = 0

        self.values = [0] * n
        self.no_echo = False

        GPIO.setup(echo_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
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
        return (delay_time * 1E6) / 58

    def mean(self):
        return float(sum(self.values)) / max(len(self.values), 1)

    def get_last_value(self):
        return self.values[self.n - 1]

    def __get_value__(self):
        log.debug("Trigger")
        self.trigger()

        start = time.time()
        c2 = GPIO.wait_for_edge(self.echo_pin, GPIO.RISING, timeout=60)
        pulse = time.time()
        c2 = GPIO.wait_for_edge(self.echo_pin, GPIO.FALLING, timeout=30)

        pulse_duration = time.time() - pulse

        if c2 is None:
            log.debug("No Echo")
            self.no_echo = True

        else:
            self.no_echo = True
            distance = self.convert_to_cm(pulse_duration)
            log.info("echo " + str(distance) + "cm")
            measurement_time = time.time() - start

            self.counter += 1

            self.values[self.counter % self.n] = distance

            if measurement_time < .06:
                log.debug("delay for next measurement")
                time.sleep(.06 - measurement_time)
            return distance


class SensorArray:
    FL = 0
    FR = 1
    LF = 1
    RF = 2
    LR = 3
    RR = 4

    trigger_pins = [1, 2, 3, 4, 5]
    echo_pins = [1, 2, 3, 4, 5]

    def __init__(self, n=6):
        self.sensors = [None] * n
        self.running = True
        self.init = 0  # Keeps count when initalising Sensors

    def add_sensor(self):
        self.sensors[self.init] = UltrasonicSensor(self.trigger_pins[self.init], self.echo_pins[self.init])
        self.init += 1

    def start_measurement_loop(self):
        self.running = True
        while self.running:
            for sensor in self.sensors:
                sensor.__get_value__()

    def stop_measurement_loop(self):
        self.running = False


if __name__ == "__main__":
    # logs sensor data on disk

    s1 = UltrasonicSensor(3, 11)

    a = SensorArray()
    a.add_sensor(s1)

    t1 = threading.Thread(target=a.start_measurement_loop)
    t1.start()
    time.sleep(1)

    while True:
        with open("Sensor.log", "a") as f:
            for sensor in a.sensors:
                f.write("{:.2f};".format(sensor.mean()))

            f.write("\n")
            time.sleep(.06 * len(a.sensors))

    a.stop_measurement_loop()
    t1.join()
    log.info("stoped gracefully")