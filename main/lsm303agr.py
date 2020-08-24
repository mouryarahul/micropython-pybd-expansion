#! /usr/bin/env python
#
# MicroPython Driver for LSM3030AGR onboard the PYBD Expansion Board.
#
# This file is part of micropython-pybd-expansion.
# https://github.com/bensherlock/micropython-pybd-expansion
#
#
# MIT License
#
# Copyright (c) 2020 Benjamin Sherlock <benjamin.sherlock@ncl.ac.uk>
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
"""MicroPython Driver for LSM3030AGR onboard the PYBD Expansion Board."""


class LSM303AGR:
    """LSM303AGR Device."""

    # Device I2C Addreses
    # Accelerometer default 7-bit address: 0011001b 0x19
    # Magnetometer default 7-bit address:  0011110b 0x1E
    ACCEL_ADDR = 0b0011001
    MAG_ADDR = 0b0011110

    # Registers and addresses from https://www.st.com/resource/en/datasheet/lsm303agr.pdf

    # Reserved 0x00-0x06
    STATUS_REG_AUX_A            = 0x07
    # Reserved 0x08-0x0B
    OUT_TEMP_L_A                = 0x0C
    OUT_TEMP_H_A                = 0x0D
    INT_COUNTER_REG_A           = 0x0E
    WHO_AM_I_A                  = 0x0F
    # Reserved 0x10-0x1E
    TEMP_CFG_REG_A              = 0x1F
    CTRL_REG1_A                 = 0x20
    CTRL_REG2_A                 = 0x21
    CTRL_REG3_A                 = 0x22
    CTRL_REG4_A                 = 0x23
    CTRL_REG5_A                 = 0x24
    CTRL_REG6_A                 = 0x25
    DATACAPTURE_A               = 0x26
    STATUS_REG_A                = 0x27
    OUT_X_L_A                   = 0x28
    OUT_X_H_A                   = 0x29
    OUT_Y_L_A                   = 0x2A
    OUT_Y_H_A                   = 0x2B
    OUT_Z_L_A                   = 0x2C
    OUT_Z_H_A                   = 0x2D
    FIFO_CTRL_REG_A             = 0x2E
    FIFO_SRC_REG_A              = 0x2F
    INT1_CFG_A                  = 0x30
    INT1_SRC_A                  = 0x31
    INT1_THS_A                  = 0x32
    INT1_DURATION_A             = 0x33
    INT2_CFG_A                  = 0x34
    INT2_SRC_A                  = 0x35
    INT2_THS_A                  = 0x36
    INT2_DURATION_A             = 0x37
    CLICK_CFG_A                 = 0x38
    CLICK_SRC_A                 = 0x39
    CLICK_THS_A                 = 0x3A
    TIME_LIMIT_A                = 0x3B
    TIME_LATENCY_A              = 0x3C
    TIME_WINDOW_A               = 0x3D
    Act_THS_A                   = 0x3E
    Act_DUR_A                   = 0x3F
    # Reserved 0x40-0x44
    OFFSET_X_REG_L_M            = 0x45
    OFFSET_X_REG_H_M            = 0x46
    OFFSET_Y_REG_L_M            = 0x47
    OFFSET_Y_REG_H_M            = 0x48
    OFFSET_Z_REG_L_M            = 0x49
    OFFSET_Z_REG_H_M            = 0x4A
    # Reserved 0x4B-0x4C
    WHO_AM_I_M                  = 0x4F
    # Reserved 0x50-0x5F
    CFG_REG_A_M                 = 0x60
    CFG_REG_B_M                 = 0x61
    CFG_REG_C_M                 = 0x62
    INT_CRTL_REG_M              = 0x63
    INT_SOURCE_REG_M            = 0x64
    INT_THS_L_REG_M             = 0x65
    INT_THS_H_REG_M             = 0x66
    STATUS_REG_M                = 0x67
    OUTX_L_REG_M                = 0x68
    OUTX_H_REG_M                = 0x69
    OUTY_L_REG_M                = 0x6A
    OUTY_H_REG_M                = 0x6B
    OUTZ_L_REG_M                = 0x6C
    OUTZ_H_REG_M                = 0x6D
    # Reserved 0x6E-0x6F

    def __init__(self, i2c = None):
        self._i2c_address = None
        self._i2c = i2c

    def __call__(self):
        return self

    # Reading and Writing to Registers. I2C See page 38 of datasheet.
    def read_byte_from_register(self, i2c_address: int, register_address: int) -> bytes:
        """Read a byte from a starting register.
        Returns a bytes object."""

        return self._i2c.readfrom_mem(i2c_address, register_address, 1)

    def read_bytes_from_register(self, i2c_address: int, register_address: int, num_bytes: int) -> bytes:
        """Read a num_bytes bytes from a starting register.
        Returns a bytes object."""
        # 8-bit register_address needs the MSB set to 1 if more than one byte is to be read.
        the_reg_address_with_flags = register_address
        if num_bytes > 1:
            the_reg_address_with_flags = the_reg_address_with_flags | 0x80

        return self._i2c.readfrom_mem(i2c_address, the_reg_address_with_flags, num_bytes)

    def write_byte_to_register(self, i2c_address: int, register_address: int, the_bytes: bytes):
        """Write the_bytes[0] to a starting register.
        Returns None."""

        self._i2c.writeto_mem(i2c_address, register_address, [the_bytes[0]])

    def write_bytes_to_register(self, i2c_address: int, register_address: int, the_bytes: bytes):
        """Write the_bytes to a starting register.
        Returns None."""
        # 8-bit register_address needs the MSB set to 1 if more than one byte is to be read.
        the_reg_address_with_flags = register_address
        if len(the_bytes) > 1:
            the_reg_address_with_flags = the_reg_address_with_flags | 0x80

        if the_bytes:
            self._i2c.writeto_mem(i2c_address, the_reg_address_with_flags, the_bytes)

        return None

    # Temperature Sensor. See page 34 of datasheet.
    # def enable_temperature_sensor(self):
    # def disable_temperature_sensor(self):
    # def read_temperature_sensor(self):

    def ben_debug(self):
        the_bytes = self.read_byte_from_register(self.ACCEL_ADDR, self.WHO_AM_I_A)
        print("WHO_AM_I_A=" + str(the_bytes[0]))

        the_bytes = self.read_byte_from_register(self.MAG_ADDR, self.WHO_AM_I_M)
        print("WHO_AM_I_M=" + str(the_bytes[0]))
