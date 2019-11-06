import RPi.GPIO as GPIO
import math

TICKS_PER_REVOLUTION = 192
WHEEL_DIAMETER = 0.1


class WheelEncoder:

    def __init__(self, pin):
        """
        Wheel Encoder sensor.
        Originally written for DAGU RS030 Hall effect wheel encoder sensor

        :param pin: The Raspberry Pi BCM pin number on which the sensor's pulse
                pin in connected
        :param wheel_diameter_metres: Wheel diameter in metres
        :param ticks_per_revolution: How many pulses the sensor gives in 1 full
                wheel revolution
        """
        self._ticks_per_revolution = TICKS_PER_REVOLUTION
        self._wheel_diameter = WHEEL_DIAMETER
        self._distance_per_tick = math.pi * self._wheel_diameter / self._ticks_per_revolution

        self._ticks = 0
        self._distance = 0.0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin, GPIO.RISING, callback=self._encoder_tick)

    def _encoder_tick(self, pin):
        self._ticks += 1
        self._distance += self._distance_per_tick

    def clear(self):
        self._ticks = 0
        self._distance = 0

    def get_ticks(self):
        return self._ticks

    def get_distance(self):
        return self._distance
