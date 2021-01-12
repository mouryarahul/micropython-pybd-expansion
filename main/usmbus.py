#! /usr/bin/env python
#
# A wrapper to provide methods of the CPython 'smbus' module on micropython.
# Provides an 'SMBus' module which supports some of the py-smbus i2c methods,
# as well as being a subclass of machine.I2C
#
# This file is part of micropyboard:
# https://github.com/mouryarahul/USMART-IOT/tree/master/micropyboard
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
"""MicroPython Wrapper for the CPython smbus over I2C."""


class SMBus:
    """
    Provides an 'SMBus' module which supports some of the py-smbus
    i2c methods, as well as being a subclass of machine.I2C
    Use it like you would the machine.I2C class:
            from machine import Pin
            from machine import I2C
            from usmbus import SMBus

            # Initialize I2C bus
            Pin('PULL_SCL', Pin.OUT, value=1) # enable 5.6kOhm X9/SCL pull-up
            Pin('PULL_SDA', Pin.OUT, value=1) # enable 5.6kOhm X10/SDA pull-up
            i2c = I2C('X', freq=400000)
            bus = SMBus(i2c)
            bus.read_byte_data(addr, register)
            ... etc
    """

    def __init__(self, i2c_bus):
        self.bus = i2c_bus

    def __call__(self):
        return self

    def scan_bus(self):
        return self.bus.scan()

    def read_byte(self, addr):
        """
        Read a single byte from a device.
        Parameters:
        addr (int) – i2c address
        Returns:
        Read byte value
        """
        return self.bus.readfrom(addr, 1)

    def write_byte(self, addr, value):
        """
        Write a single byte to a device.

        Parameters:
        addr (int) – i2c address
        value (int) – value to write
        """
        # writeto() expects something it can treat as a buffer
        if isinstance(value, int):
            value = bytes([value])

        return self.bus.writeto(addr, value)

    def read_word_data(self, addr, register):
        """
        Read a single word (2 bytes) from a given register.
        Parameters
        addr (int) – i2c address
        register (int) – Register to read
        Returns 2-byte word
        Return type int
        """
        two_bytes = self.bus.readfrom_mem(addr, register, 2)
        return two_bytes[1] << 8 | two_bytes[0]

    def write_word_data(self, addr, register, value):
        """
        Write a single word (2 bytes) to a given register.
        Parameters
        addr (int) – i2c address
        register (int) – Register to read
        Return None
        """
        # writeto_mem() expects something it can treat as a buffer
        if isinstance(value, int):
            value = bytes([(value & 0xff00) >> 8]) + bytes([value & 0xff])

        self.bus.writeto_mem(addr, register, value)

    def read_byte_data(self, addr, register):
        """
        Read a single byte from a designated register.
        Parameters
        addr (int) – i2c address
        register (int) – Register to read
        Returns Read byte value
        Return type int
        """
        return self.bus.readfrom_mem(addr, register, 1)[0]

    def write_byte_data(self, addr, register, value):
        """
        Write a single byte to a device.
        Parameters
        addr (int) – i2c address
        value (int) – value to write
        """
        # writeto_mem() expects something it can treat as a buffer
        if isinstance(value, int):
            value = bytes([value])

        return self.bus.writeto_mem(addr, register, value)

    def read_i2c_block_data(self, addr, register, length):
        """
        Read a block of byte data from a given register.
        Parameters
        addr (int) – i2c address
        register (int) – Start register
        length (int) – Desired block length
        Returns List of bytes
        Return type list
        """
        return self.bus.readfrom_mem(addr, register, length)

    def write_i2c_block_data(self, addr, register, data):
        """ Write multiple bytes of data to register of device at addr
            Returns None """
        # writeto_mem() expects something it can treat as a buffer
        if isinstance(data, int):
            data = bytes([data])

        return self.bus.writeto_mem(addr, register, data)

