#!/usr/bin/python

import threading
import time, random
import logging
import RPi.GPIO as GPIO
import math

logging.basicConfig(level=logging.CRITICAL)

log = logging.getLogger(__name__)
GPIO.setmode(GPIO.BOARD)


class Switch:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)


    def read_pin(self):
        return GPIO.input(self.pin)

    def write_pin(self, state):
        GPIO.output(self. pin, state)

    def left_course(self):
        return self.read_pin() == 1

    def right_course(self):
        return self.read_pin() == 0

class Display:
        """ MSB first!!!!"""

        def __init__(self, pins):
            self.pins = pins
            GPIO.setup(self.pins, GPIO.IN, pull_up_down = GPIO.PUD_UP)
            GPIO.output(pins, GPIO.LOW)
            self.display_number(0)

        def translate_to_bin_digit(self, number, index):
            binary = bin(number)[-2:]
            return int(binary[index])


        def display_number(self, number):

            tr = self.translate_to_bin_digit

            GPIO.output(self.pins, (tr(number,0), tr(number,1), tr(number,2), tr(number,3)))







if __name__ == "__main__":
    course_switch = Switch(0)
    d = Display

    d.display_number("6")

    while True:
        print course_switch.read_pin()