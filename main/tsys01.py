#! /usr/bin/env python
#
# A micropython module to interface with MS5837-30BA and MS5837-02BA
# waterproof pressure and temperature sensors from BlueRobotics
#
# This file is part of micropyboard derived from Python Driver.
# https://github.com/bluerobotics/tsys01-python
#
# MIT License
#
# Copyright (c) 2020 Rahul Mourya <mourya.rahul1981@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Example to usage of utsys01:
# from machine import Pin
# from machine import I2C
# from tsys01 import TSYS01
#
# Initialize I2C bus
# Pin('PULL_SCL', Pin.OUT, value=1) # enable 5.6kOhm X9/SCL pull-up
# Pin('PULL_SDA', Pin.OUT, value=1) # enable 5.6kOhm X10/SDA pull-up
# try:
#    i2c = I2C('X', freq=400000)
# except Exception as error:
#    print(error.__class__.__name__ + ": " + str(error))
#
# Instantiate Sensor TSYS01
# try:
# 	 tsys01 = TSYS01(i2c)
# except Exception as error:
#    print(error.__class__.__name__ + ": " + str(error))
#
"""MicroPython driver for the TSYS01 over I2C Y."""

from time import sleep
from pybd_expansion.main.usmbus import SMBus

# Valid units
UNITS_Centigrade = 1
UNITS_Farenheit = 2
UNITS_Kelvin = 3


class TSYS01:
    # Registers
    _TSYS01_ADDR = 0x77
    _TSYS01_PROM_READ = 0xA0
    _TSYS01_RESET = 0x1E
    _TSYS01_CONVERT = 0x48
    _TSYS01_READ = 0x00

    def __init__(self, i2c):
        # Degrees C
        self._temperature = 0
        self._k = []

        try:
            self._bus = SMBus(i2c)
        except Exception as error:
            print(error.__class__.__name__ + ": " + str(error))
            self._bus = None

    def __call__(self):
        return self

    def init(self):
        if self._bus is None:
            print("No bus!")
            return False

        self._bus.write_byte(self._TSYS01_ADDR, self._TSYS01_RESET)

        # Wait for reset to complete
        sleep(0.1)

        self._k = []

        # Read calibration values
        # Read one 16 bit byte word at a time
        for prom in range(0xAA, 0xA2 - 2, -2):
            k = self._bus.read_word_data(self._TSYS01_ADDR, prom)
            k = ((k & 0xFF) << 8) | (k >> 8)  # SMBus is little-endian for word transfers, we need to swap MSB and LSB
            self._k.append(k)

        return True

    def read(self):
        if self._bus is None:
            print("No bus!")
            return False

        # Request conversion
        self._bus.write_byte(self._TSYS01_ADDR, self._TSYS01_CONVERT)

        # Max conversion time = 9.04 ms
        sleep(0.01)

        adc = self._bus.read_i2c_block_data(self._TSYS01_ADDR, self._TSYS01_READ, 3)
        adc = adc[0] << 16 | adc[1] << 8 | adc[2]
        self._calculate(adc)
        return True

    # Temperature in requested units
    # default degrees C
    def temperature(self, conversion=UNITS_Centigrade):
        if conversion == UNITS_Farenheit:
            return (9 / 5) * self._temperature + 32
        elif conversion == UNITS_Kelvin:
            return self._temperature - 273
        return self._temperature

    # Cribbed from datasheet
    def _calculate(self, adc):
        adc16 = adc / 256
        self._temperature = -2 * self._k[4] * 10 ** -21 * adc16 ** 4 + \
                            4 * self._k[3] * 10 ** -16 * adc16 ** 3 + \
                            -2 * self._k[2] * 10 ** -11 * adc16 ** 2 + \
                            1 * self._k[1] * 10 ** -6 * adc16 + \
                            -1.5 * self._k[0] * 10 ** -2
