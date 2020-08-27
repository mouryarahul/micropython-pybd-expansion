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
_MAGNETO_I2C_ADDR = 0b0011110


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
# Data Rates
ACCEL_ODR_POWERDOWN             = 0b0000
ACCEL_ODR_1HZ                   = 0b0001
ACCEL_ODR_10HZ                  = 0b0010
ACCEL_ODR_25HZ                  = 0b0011
ACCEL_ODR_50HZ                  = 0b0100
ACCEL_ODR_100HZ                 = 0b0101
ACCEL_ODR_200HZ                 = 0b0110
ACCEL_ODR_400HZ                 = 0b0111
ACCEL_ODR_1620HZ                = 0b1000
ACCEL_ODR_1344HZ                = 0b1001

# Full Scale
ACCEL_FS_2G                     = 0b00
ACCEL_FS_4G                     = 0b01
ACCEL_FS_8G                     = 0b10
ACCEL_FS_16G                    = 0b11

# Magnetometer System Mode, see Table 94.
MAGNETO_MD_CONTINUOUS           = 0b00
MAGNETO_MD_SINGLE               = 0b01
MAGNETO_MD_IDLE0                = 0b10
MAGNETO_MD_IDLE1                = 0b11

# Magnetometer Output Data Rate
MAGNETO_ODR_10HZ                = 0b00
MAGNETO_ODR_20HZ                = 0b01
MAGNETO_ODR_50HZ                = 0b10
MAGNETO_ODR_100HZ               = 0b11


class LSM303AGR:
    """LSM303AGR Device."""

    # Device I2C Addreses
    # Accelerometer default 7-bit address: 0011001b 0x19
    # Magnetometer default 7-bit address:  0011110b 0x1E

    def __init__(self, i2c=None, accel_i2c_address=_ACCEL_I2C_ADDR, magneto_i2c_address=_MAGNETO_I2C_ADDR):
        self._accel_i2c_address = accel_i2c_address
        self._magneto_i2c_address = magneto_i2c_address
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

        the_bytes = self._read_byte_from_register(_MAGNETO_I2C_ADDR, _REG_WHO_AM_I_M)
        if not the_bytes:
            return False
        whoami_m = the_bytes[0]

        if whoami_a != _EXPVAL_WHO_AM_I_A:
            return False

        if whoami_m != _EXPVAL_WHO_AM_I_M:
            return False

        return True

    # Temperature Sensor. See page 34 of datasheet.

    def set_temp_cfg_reg(self, en=0):
        """Set Temperature TEMP_CFG_REG values."""
        reg_val = ((en & 0x01) << 6) | ((en & 0x01) << 7)
        self._write_bytes_to_register(self._accel_i2c_address, _REG_TEMP_CFG_REG_A, bytes([reg_val]))
        return None

    def get_temp_cfg_reg(self):
        """Get Temperature TEMP_CFG_REG values."""
        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_TEMP_CFG_REG_A)
        reg_val = the_bytes[0]
        en = (reg_val >> 7) & 0x01
        return reg_val, en

    def get_temp_status_reg_aux(self):
        """Get Temperature STATUS_REG_AUX values."""
        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_STATUS_REG_AUX_A)
        reg_val = the_bytes[0]
        tor = (reg_val >> 6) & 0x01
        tda = (reg_val >> 2) & 0x01
        return reg_val, tda, tor

    def get_temp_output_int(self):
        """Get Temperature output from the OUT_TEMP_L_A to OUT_TEMP_H_A registers.
        Returns value as int direct from the upper register only."""
        # Upper byte is signed int8 offset from 25degreesC. In high resolution mode, the lower byte is decimal.
        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_OUT_TEMP_L_A, 2)
        temp_val = self._bytes_to_int8(the_bytes[1]) + 25
        return temp_val

    def get_temp_output_float(self):
        """Get Temperature output from the OUT_TEMP_L_A to OUT_TEMP_H_A registers.
        Returns value as float using full precision from both registers.
        Note that Low Power/Normal/High Precision modes affect the bit width of the underlying temperature data."""
        # Upper byte is signed int8 offset from 25degreesC. In high resolution mode, the lower byte is decimal.
        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_OUT_TEMP_L_A, 2)
        temp_val = self._bytes_to_int16(the_bytes[0], the_bytes[1]) + (25 << 8)
        temp_val_float = float(temp_val) / 256.0
        return temp_val_float

    # Accelerometer

    def set_accel_ctrl_reg1(self, xen=1, zen=1, yen=1, lpen=0, accel_odr=ACCEL_ODR_POWERDOWN):
        """Set Accelerometer CTRL_REG1 values."""
        reg_val = ((accel_odr & 0x0F) << 4) | ((lpen & 0x01) << 3) | ((zen & 0x01) << 2) | ((yen & 0x01) << 1) | (xen & 0x01)
        self._write_bytes_to_register(self._accel_i2c_address, _REG_CTRL_REG1_A, bytes([reg_val]))
        return None

    def get_accel_ctrl_reg1(self):
        """Get the Accelerometer CTRL_REG1 values."""
        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_CTRL_REG1_A)
        reg_val = the_bytes[0]
        xen = reg_val & 0x01
        yen = (reg_val >> 1) & 0x01
        zen = (reg_val >> 2) & 0x01
        lpen = (reg_val >> 3) & 0x01
        odr = (reg_val >> 4) & 0x0F
        return reg_val, xen, yen, zen, lpen, odr

    def set_accel_ctrl_reg4(self, hr=0, accel_fs=ACCEL_FS_2G, bdu=0):
        """Set Accelerometer CTRL_REG4 values."""
        reg_val = ((bdu & 0x01) << 7) | ((accel_fs & 0x03) << 4) | ((hr & 0x01) << 3)
        self._write_bytes_to_register(self._accel_i2c_address, _REG_CTRL_REG4_A, bytes([reg_val]))

    def get_accel_ctrl_reg4(self):
        """Get the Accelerometer CTRL_REG4 values."""
        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_CTRL_REG4_A)
        reg_val = the_bytes[0]
        hr = (reg_val >> 3) & 0x01
        fs = (reg_val >> 4) & 0x03
        bdu = (reg_val >> 7) & 0x01
        return reg_val, hr, fs, bdu

    def get_accel_status_reg(self):
        """Get the Accelerometer STATUS_REG values."""
        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_STATUS_REG_A)
        reg_val = the_bytes[0]
        xda = reg_val & 0x01
        yda = (reg_val >> 1) & 0x01
        zda = (reg_val >> 2) & 0x01
        zyxda = (reg_val >> 3) & 0x01
        xor = (reg_val >> 4) & 0x01
        yor = (reg_val >> 5) & 0x01
        zor = (reg_val >> 6) & 0x01
        zyxor = (reg_val >> 7) & 0x01
        return reg_val, xda, yda, zda, zyxda, xor, yor, zor, zyxor

    def get_accel_outputs(self):
        """Get Accelerometer outputs for X, Y and Z axis from the OUT_X_L_A to OUT_Z_H_A registers.
        Returns x, y, z as int16 direct from the registers."""
        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_OUT_X_L_A, 6)
        x_val = self._bytes_to_int16(the_bytes[0], the_bytes[1])
        y_val = self._bytes_to_int16(the_bytes[2], the_bytes[3])
        z_val = self._bytes_to_int16(the_bytes[4], the_bytes[5])

        return x_val, y_val, z_val

    # Magnetometer

    def set_magneto_cfg_reg_a(self, magneto_md=MAGNETO_MD_CONTINUOUS, magneto_odr=MAGNETO_ODR_10HZ, lp=0, soft_rst=0, reboot=0, comp_temp_en=0):
        """Set Magnetometer CFG_REG_A values."""
        reg_val = ((comp_temp_en & 0x01) << 7) | ((reboot & 0x01) << 6) | ((soft_rst & 0x01) << 5) | ((lp & 0x01) << 4) | ((magneto_odr & 0x03) << 2) | (magneto_md & 0x03)
        self._write_bytes_to_register(self._magneto_i2c_address, _REG_CFG_REG_A_M, bytes([reg_val]))
        return None

    def get_magneto_cfg_reg_a(self):
        """Get the Magnetometer CFG_REG_A values."""
        the_bytes = self._read_bytes_from_register(self._magneto_i2c_address, _REG_CFG_REG_A_M)
        reg_val = the_bytes[0]
        md = reg_val & 0x03
        odr = (reg_val >> 2) & 0x03
        lp = (reg_val >> 4) & 0x01
        soft_rst = (reg_val >> 5) & 0x01
        reboot = (reg_val >> 6) & 0x01
        comp_temp_en = (reg_val >> 7) & 0x01
        return reg_val, md, odr, lp, soft_rst, reboot, comp_temp_en

    def set_magneto_cfg_reg_b(self, lpf=0, off_canc=0, set_freq=0, int_on_dataoff=0, off_canc_one_shot=0):
        """Set Magnetometer CFG_REG_B values."""
        reg_val = ((off_canc_one_shot & 0x01) << 4) | ((int_on_dataoff & 0x01) << 3) | ((set_freq & 0x01) << 2) | ((off_canc & 0x01) << 1) | (lpf & 0x01)
        self._write_bytes_to_register(self._magneto_i2c_address, _REG_CFG_REG_B_M, bytes([reg_val]))
        return None

    def get_magneto_cfg_reg_b(self):
        """Get the Magnetometer CFG_REG_B values."""
        the_bytes = self._read_bytes_from_register(self._magneto_i2c_address, _REG_CFG_REG_B_M)
        reg_val = the_bytes[0]
        lpf = reg_val & 0x01
        off_canc = (reg_val >> 1) & 0x01
        set_freq = (reg_val >> 2) & 0x01
        int_on_dataoff = (reg_val >> 3) & 0x01
        off_canc_one_shot = (reg_val >> 4) & 0x01
        return reg_val, lpf, off_canc, set_freq, int_on_dataoff, off_canc_one_shot

    def set_magneto_cfg_reg_c(self, int_mag=0, self_test=0, ble=0, bdu=0, int_mag_pin=0):
        """Set Magnetometer CFG_REG_C values."""
        reg_val = ((int_mag_pin & 0x01) << 6) | ((bdu & 0x01) << 4) | ((ble & 0x01) << 3) | ((self_test & 0x01) << 1) | (int_mag & 0x01)
        self._write_bytes_to_register(self._magneto_i2c_address, _REG_CFG_REG_C_M, bytes([reg_val]))
        return None

    def get_magneto_cfg_reg_c(self):
        """Get the Magnetometer CFG_REG_C values."""
        the_bytes = self._read_bytes_from_register(self._magneto_i2c_address, _REG_CFG_REG_C_M)
        reg_val = the_bytes[0]
        int_mag = reg_val & 0x01
        self_test = (reg_val >> 1) & 0x01
        ble = (reg_val >> 3) & 0x01
        bdu = (reg_val >> 4) & 0x01
        int_mag_pin = (reg_val >> 6) & 0x01
        return reg_val, int_mag, self_test, ble, bdu, int_mag_pin

    def get_magneto_status_reg(self):
        """Get the Magnetometer STATUS_REG values."""
        the_bytes = self._read_bytes_from_register(self._magneto_i2c_address, _REG_STATUS_REG_M)
        reg_val = the_bytes[0]
        xda = reg_val & 0x01
        yda = (reg_val >> 1) & 0x01
        zda = (reg_val >> 2) & 0x01
        zyxda = (reg_val >> 3) & 0x01
        xor = (reg_val >> 4) & 0x01
        yor = (reg_val >> 5) & 0x01
        zor = (reg_val >> 6) & 0x01
        zyxor = (reg_val >> 7) & 0x01
        return reg_val, xda, yda, zda, zyxda, xor, yor, zor, zyxor

    def get_magneto_outputs(self):
        """Get Magnetometer outputs for X, Y and Z axis from the OUTX_L_REG_M to OUTZ_H_REG_M registers.
        Returns x, y, z as int16 direct from the registers."""
        the_bytes = self._read_bytes_from_register(self._magneto_i2c_address, _REG_OUTX_L_REG_M, 6)
        x_val = self._bytes_to_int16(the_bytes[0], the_bytes[1])
        y_val = self._bytes_to_int16(the_bytes[2], the_bytes[3])
        z_val = self._bytes_to_int16(the_bytes[4], the_bytes[5])

        return x_val, y_val, z_val

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

    @staticmethod
    def _bytes_to_uint32(byte0, byte1, byte2, byte3) -> int:
        """Gets uint32 from the bytes. Where byte0 is LSB."""
        combined = byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24)

        return int(combined)

    @staticmethod
    def _bytes_to_uint16(byte0, byte1) -> int:
        """Gets uint16 from the bytes. Where byte0 is LSB."""
        combined = byte0 | (byte1 << 8)

        return int(combined)

    @staticmethod
    def _bytes_to_int16(byte0, byte1) -> int:
        """Gets int16 from the bytes. Where byte0 is LSB."""
        combined = byte0 | (byte1 << 8)
        val = int(combined)
        if val > 32767:
            val = val - 65536

        return val

    @staticmethod
    def _bytes_to_int8(byte0) -> int:
        """Gets int8 from the bytes. Where byte0 is LSB."""
        combined = byte0
        val = int(combined)
        if val > 127:
            val = val - 256

        return val

    def run_example(self):
        """A simple example showing how to initialise and run the LSM303AGR for
        Accelerormeter, Magnetometer, and Temperature readings."""
        the_bytes = self._read_bytes_from_register(_ACCEL_I2C_ADDR, _REG_WHO_AM_I_A)
        print("WHO_AM_I_A=" + str(the_bytes[0]))

        the_bytes = self._read_bytes_from_register(_MAGNETO_I2C_ADDR, _REG_WHO_AM_I_M)
        print("WHO_AM_I_M=" + str(the_bytes[0]))

        the_bytes = self._read_bytes_from_register(self._accel_i2c_address, _REG_CTRL_REG1_A)
        print("CTRL_REG1_A=" + str(the_bytes[0]))

        # Temperature
        self.set_accel_ctrl_reg4(bdu=1)
        self.set_temp_cfg_reg(en=1)

        # Accelerometer
        self.set_accel_ctrl_reg1(accel_odr=ACCEL_ODR_1HZ)
        self.set_accel_ctrl_reg4(accel_fs=ACCEL_FS_2G, hr=0, bdu=1)  # bdu=1 needed for temperature

        # Magnetometer
        self.set_magneto_cfg_reg_a(magneto_odr=MAGNETO_ODR_10HZ, magneto_md=MAGNETO_MD_CONTINUOUS, lp=0, comp_temp_en=1)
        self.set_magneto_cfg_reg_b(lpf=1)
        self.set_magneto_cfg_reg_c(bdu=1)

        while True:
            # Check the temperature status register for new data
            status_reg, tda, tor = self.get_temp_status_reg_aux()
            if tda:
                temperature = self.get_temp_output_float()
                print("temperature=" + str(temperature))

            # Check the status register for new accelerometer data
            status_reg, xda, yda, zda, zyxda, xor, yor, zor, zyxor = self.get_accel_status_reg()
            if zyxda:
                x_val, y_val, z_val = self.get_accel_outputs()
                print("Accel: x_val=" + str(x_val) + " y_val=" + str(y_val) + " z_val=" + str(z_val))

            # Check the status register for new magnetometer data
            status_reg, xda, yda, zda, zyxda, xor, yor, zor, zyxor = self.get_magneto_status_reg()
            if zyxda:
                x_val, y_val, z_val = self.get_magneto_outputs()
                print("Magneto: x_val=" + str(x_val) + " y_val=" + str(y_val) + " z_val=" + str(z_val))


def main():
    import pyb
    pyb.Pin.board.EN_3V3.off()
    pyb.delay(1000)

    pyb.Pin.board.EN_3V3.on()
    pyb.delay(500)
    pyb.Pin('PULL_SCL', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X9/SCL pull-up
    pyb.Pin('PULL_SDA', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X10/SDA pull-up
    # i2c = machine.I2C(1, freq=400000)  # machine.I2C
    i2c = pyb.I2C(1) # pyb.I2C
    i2c.init(pyb.I2C.MASTER, baudrate=400000)  # pyb.I2C

    sensor = LSM303AGR(i2c)
    sensor.run_example()


if __name__ == '__main__':
    main()
