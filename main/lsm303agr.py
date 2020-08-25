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

# Default I2C Addresses
_ACCEL_I2C_ADDR = 0b0011001
_MAG_I2C_ADDR = 0b0011110


# Registers and addresses from https://www.st.com/resource/en/datasheet/lsm303agr.pdf

# Reserved 0x00-0x06
_REG_STATUS_REG_AUX_A            = 0x07
# Reserved 0x08-0x0B
_REG_OUT_TEMP_L_A                = 0x0C
_REG_OUT_TEMP_H_A                = 0x0D
_REG_INT_COUNTER_REG_A           = 0x0E
_REG_WHO_AM_I_A                  = 0x0F
# Reserved 0x10-0x1E
_REG_TEMP_CFG_REG_A              = 0x1F
_REG_CTRL_REG1_A                 = 0x20
_REG_CTRL_REG2_A                 = 0x21
_REG_CTRL_REG3_A                 = 0x22
_REG_CTRL_REG4_A                 = 0x23
_REG_CTRL_REG5_A                 = 0x24
_REG_CTRL_REG6_A                 = 0x25
_REG_DATACAPTURE_A               = 0x26
_REG_STATUS_REG_A                = 0x27
_REG_OUT_X_L_A                   = 0x28
_REG_OUT_X_H_A                   = 0x29
_REG_OUT_Y_L_A                   = 0x2A
_REG_OUT_Y_H_A                   = 0x2B
_REG_OUT_Z_L_A                   = 0x2C
_REG_OUT_Z_H_A                   = 0x2D
_REG_FIFO_CTRL_REG_A             = 0x2E
_REG_FIFO_SRC_REG_A              = 0x2F
_REG_INT1_CFG_A                  = 0x30
_REG_INT1_SRC_A                  = 0x31
_REG_INT1_THS_A                  = 0x32
_REG_INT1_DURATION_A             = 0x33
_REG_INT2_CFG_A                  = 0x34
_REG_INT2_SRC_A                  = 0x35
_REG_INT2_THS_A                  = 0x36
_REG_INT2_DURATION_A             = 0x37
_REG_CLICK_CFG_A                 = 0x38
_REG_CLICK_SRC_A                 = 0x39
_REG_CLICK_THS_A                 = 0x3A
_REG_TIME_LIMIT_A                = 0x3B
_REG_TIME_LATENCY_A              = 0x3C
_REG_TIME_WINDOW_A               = 0x3D
_REG_Act_THS_A                   = 0x3E
_REG_Act_DUR_A                   = 0x3F
# Reserved 0x40-0x44
_REG_OFFSET_X_REG_L_M            = 0x45
_REG_OFFSET_X_REG_H_M            = 0x46
_REG_OFFSET_Y_REG_L_M            = 0x47
_REG_OFFSET_Y_REG_H_M            = 0x48
_REG_OFFSET_Z_REG_L_M            = 0x49
_REG_OFFSET_Z_REG_H_M            = 0x4A
# Reserved 0x4B-0x4C
_REG_WHO_AM_I_M                  = 0x4F
# Reserved 0x50-0x5F
_REG_CFG_REG_A_M                 = 0x60
_REG_CFG_REG_B_M                 = 0x61
_REG_CFG_REG_C_M                 = 0x62
_REG_INT_CRTL_REG_M              = 0x63
_REG_INT_SOURCE_REG_M            = 0x64
_REG_INT_THS_L_REG_M             = 0x65
_REG_INT_THS_H_REG_M             = 0x66
_REG_STATUS_REG_M                = 0x67
_REG_OUTX_L_REG_M                = 0x68
_REG_OUTX_H_REG_M                = 0x69
_REG_OUTY_L_REG_M                = 0x6A
_REG_OUTY_H_REG_M                = 0x6B
_REG_OUTZ_L_REG_M                = 0x6C
_REG_OUTZ_H_REG_M                = 0x6D
# Reserved 0x6E-0x6F

_EXPVAL_WHO_AM_I_A              = 51
_EXPVAL_WHO_AM_I_M              = 64

# See Table 35, page 47 of datasheet
ACCEL_POWERMODE_OFF             = 0b0000
ACCEL_POWERMODE_HR_LP_1HZ       = 0b0001
ACCEL_POWERMODE_HR_LP_10HZ      = 0b0010
ACCEL_POWERMODE_HR_LP_25HZ      = 0b0011
ACCEL_POWERMODE_HR_LP_50HZ      = 0b0100
ACCEL_POWERMODE_HR_LP_100HZ     = 0b0101
ACCEL_POWERMODE_HR_LP_200HZ     = 0b0110
ACCEL_POWERMODE_HR_LP_400HZ     = 0b0111
ACCEL_POWERMODE_LP_1620HZ       = 0b1000
ACCEL_POWERMODE_HR_1344HZ       = 0b1001


class LSM303AGR:
    """LSM303AGR Device."""

    # Device I2C Addreses
    # Accelerometer default 7-bit address: 0011001b 0x19
    # Magnetometer default 7-bit address:  0011110b 0x1E

    def __init__(self, i2c=None, accel_i2c_address=_ACCEL_I2C_ADDR, mag_i2c_address=_MAG_I2C_ADDR):
        self._accel_i2c_address = accel_i2c_address
        self._mag_i2c_address = mag_i2c_address
        self._i2c = i2c

    def __call__(self):
        return self

    def test_whoami(self):
        """Test the Sensor WHO_AM_I_A and WHO_AM_I_M register values to confirm device is powered and communicable.
        Returns True if all ok. False if not."""
        the_bytes = self._read_byte_from_register(_ACCEL_I2C_ADDR, _REG_WHO_AM_I_A)
        if not the_bytes:
            return False
        whoami_a = the_bytes[0]

        the_bytes = self._read_byte_from_register(_MAG_I2C_ADDR, _REG_WHO_AM_I_M)
        if not the_bytes:
            return False
        whoami_m = the_bytes[0]

        if whoami_a != _EXPVAL_WHO_AM_I_A:
            return False

        if whoami_m != _EXPVAL_WHO_AM_I_M:
            return False

        return True

    # Temperature Sensor. See page 34 of datasheet.
    # def enable_temperature_sensor(self):
    # def disable_temperature_sensor(self):
    # def read_temperature_sensor(self):

    # Accelerometer Power Modes
    def set_accel_power_mode(self, accel_powermode):
        # Read current register value to get other bits.
        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_CTRL_REG1_A)
        reg_val = (the_bytes[0] & 0x0F) | ((accel_powermode & 0x0F) << 4)

        self._write_bytes_to_register(self._accel_i2c_address, _REG_CTRL_REG1_A, bytes([reg_val]) )


    # Reading and Writing to Registers. I2C See page 38 of datasheet.
    def _read_bytes_from_register(self, i2c_address: int, register_address: int, num_bytes: int = 1) -> bytes:
        """Read a num_bytes bytes from a starting register.
        Returns a bytes object."""
        # 8-bit register_address needs the MSB set to 1 if more than one byte is to be read.
        the_reg_address_with_flags = register_address
        if num_bytes > 1:
            the_reg_address_with_flags = the_reg_address_with_flags | 0x80

        # return self._i2c.readfrom_mem(i2c_address, the_reg_address_with_flags, num_bytes) # machine.I2C
        return self._i2c.mem_read(num_bytes, i2c_address, the_reg_address_with_flags)  # pyb.I2C


    def _write_bytes_to_register(self, i2c_address: int, register_address: int, the_bytes: bytes):
        """Write the_bytes to a starting register.
        Returns None."""
        # 8-bit register_address needs the MSB set to 1 if more than one byte is to be read.
        the_reg_address_with_flags = register_address
        if len(the_bytes) > 1:
            the_reg_address_with_flags = the_reg_address_with_flags | 0x80

        if the_bytes:
            # self._i2c.writeto_mem(i2c_address, the_reg_address_with_flags, the_bytes) # machine.I2C
            self._i2c.mem_write(the_bytes, i2c_address, the_reg_address_with_flags) # pyb.I2C

        return None



    def ben_debug(self):
        the_bytes = self._read_bytes_from_register(_ACCEL_I2C_ADDR, _REG_WHO_AM_I_A)
        print("WHO_AM_I_A=" + str(the_bytes[0]))

        the_bytes = self._read_bytes_from_register(_MAG_I2C_ADDR, _REG_WHO_AM_I_M)
        print("WHO_AM_I_M=" + str(the_bytes[0]))

        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_CTRL_REG1_A)
        print("CTRL_REG1_A=" + str(the_bytes[0]))

        import pyb
        self.set_accel_power_mode(ACCEL_POWERMODE_OFF)

        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_CTRL_REG1_A)
        print("CTRL_REG1_A=" + str(the_bytes[0]))


def main():
    import pyb
    pyb.Pin.board.EN_3V3.on()
    pyb.delay(20)
    pyb.Pin('PULL_SCL', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X9/SCL pull-up
    pyb.Pin('PULL_SDA', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X10/SDA pull-up
    # i2c = machine.I2C(1, freq=400000)  # machine.I2C
    i2c = pyb.I2C(1) # pyb.I2C
    i2c.init(pyb.I2C.MASTER, baudrate=400000) # pyb.I2C


    sensor=LSM303AGR(i2c)
    sensor.ben_debug()


if __name__ == '__main__':
    main()