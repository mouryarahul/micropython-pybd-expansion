#! /usr/bin/env python
#
# MicroPython Driver for BME280 onboard the PYBD Expansion Board.
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
"""MicroPython Driver for BME280 onboard the PYBD Expansion Board."""

# Default I2C Address - SDO tied to GND
_BME280_I2C_ADDR = 0b1110110

# Registers and addresses from https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280-DS002.pdf

# Reserved 0x00-0x87
_REG_CALIB00            = 0x88
_REG_CALIB01            = 0x89
_REG_CALIB02            = 0x8A
_REG_CALIB03            = 0x8B
_REG_CALIB04            = 0x8C
_REG_CALIB05            = 0x8D
_REG_CALIB06            = 0x8E
_REG_CALIB07            = 0x8F
_REG_CALIB08            = 0x90
_REG_CALIB09            = 0x91
_REG_CALIB10            = 0x92
_REG_CALIB11            = 0x93
_REG_CALIB12            = 0x94
_REG_CALIB13            = 0x95
_REG_CALIB14            = 0x96
_REG_CALIB15            = 0x97
_REG_CALIB16            = 0x98
_REG_CALIB17            = 0x99
_REG_CALIB18            = 0x9A
_REG_CALIB19            = 0x9B
_REG_CALIB20            = 0x9C
_REG_CALIB21            = 0x9D
_REG_CALIB22            = 0x9E
_REG_CALIB23            = 0x9F
_REG_CALIB24            = 0xA0
_REG_CALIB25            = 0xA1
# Reserved 0xA2-AxCF
_REG_ID                 = 0xD0
# Reserved 0xD1-0xDF
_REG_RESET              = 0xE0
_REG_CALIB26            = 0xE1
_REG_CALIB27            = 0xE2
_REG_CALIB28            = 0xE3
_REG_CALIB29            = 0xE4
_REG_CALIB30            = 0xE5
_REG_CALIB31            = 0xE6
_REG_CALIB32            = 0xE7
_REG_CALIB33            = 0xE8
_REG_CALIB34            = 0xE9
_REG_CALIB35            = 0xEA
_REG_CALIB36            = 0xEB
_REG_CALIB37            = 0xEC
_REG_CALIB38            = 0xED
_REG_CALIB39            = 0xEE
_REG_CALIB40            = 0xEF
_REG_CALIB41            = 0xF0
# Reserved 0xF1
_REG_CTRL_HUM           = 0xF2
_REG_STATUS             = 0xF3
_REG_CTRL_MEAS          = 0xF4
_REG_CONFIG             = 0xF5
# Reserved 0xF6
_REG_PRESS_MSB          = 0xF7
_REG_PRESS_LSB          = 0xF8
_REG_PRESS_XLSB         = 0xF9
_REG_TEMP_MSB           = 0xFA
_REG_TEMP_LSB           = 0xFB
_REG_TEMP_XLSB          = 0xFC
_REG_HUM_MSB            = 0xFD
_REG_HUM_LSB            = 0xFE

_EXPVAL_ID              = 0x60


# See Table 20, 23, 24
OSRS_SKIPPED            = 0b000
OSRS_OVERSAMPLE_X_1     = 0b001
OSRS_OVERSAMPLE_X_2     = 0b010
OSRS_OVERSAMPLE_X_4     = 0b011
OSRS_OVERSAMPLE_X_8     = 0b100
OSRS_OVERSAMPLE_X_16    = 0b101

# See Table 25
MODE_SLEEP              = 0b00
MODE_FORCED             = 0b01
MODE_NORMAL             = 0b11

# See Table 27
STANDBY_T_0_5_MS        = 0b000
STANDBY_T_62_5_MS       = 0b001
STANDBY_T_125_MS        = 0b010
STANDBY_T_250_MS        = 0b011
STANDBY_T_500_MS        = 0b100
STANDBY_T_1000_MS       = 0b101
STANDBY_T_10_MS         = 0b110
STANDBY_T_20_MS         = 0b111

# See Table 28
FILTER_COEF_OFF         = 0b000
FILTER_COEF_2           = 0b001
FILTER_COEF_4           = 0b010
FILTER_COEF_8           = 0b011
FILTER_COEF_16          = 0b100


class BME280:
    """BME280 Device."""

    def __init__(self, i2c=None, i2c_address=_BME280_I2C_ADDR):
        self._i2c_address = i2c_address
        self._i2c = i2c
        # Calibration parameters
        self._calibration_parameters_retrieved = False
        self._dig_T1 = 0  # uint16
        self._dig_T2 = 0  # int16
        self._dig_T3 = 0  # int16
        self._dig_P1 = 0  # uint16
        self._dig_P2 = 0  # int16
        self._dig_P3 = 0  # int16
        self._dig_P4 = 0  # int16
        self._dig_P5 = 0  # int16
        self._dig_P6 = 0  # int16
        self._dig_P7 = 0  # int16
        self._dig_P8 = 0  # int16
        self._dig_P9 = 0  # int16
        self._dig_H1 = 0  # uint8
        self._dig_H2 = 0  # int16
        self._dig_H3 = 0  # uint8
        self._dig_H4 = 0  # int16
        self._dig_H5 = 0  # int16
        self._dig_H6 = 0  # int8

        self._temperature_for_compensation = 0.0

    def __call__(self):
        return self

    def test_id(self):
        """Test the Sensor ID to confirm device is powered and communicable.
        Returns True if all ok. False if not."""
        the_bytes = self._read_bytes_from_register(self._i2c_address, _REG_ID, 1)
        if not the_bytes:
            return False
        the_id = the_bytes[0]

        if the_id != _EXPVAL_ID:
            return False

        return True

    def soft_reset_device(self):
        """Soft reset the device by sending the reset command via I2C."""
        reg_val = 0xB6
        self._write_bytes_to_register(self._i2c_address, _REG_RESET, bytes([reg_val]))
        return None

    def retrieve_calibration_parameters(self):
        """Retrieve the calibration parameters from the device and store locally for use in compensation."""
        # Table 16
        the_bytes = self._read_bytes_from_register(self._i2c_address, _REG_CALIB00, 26)  # First run _REG_CALIB00 to _REG_CALIB25
        self._dig_T1 = (the_bytes[1] << 8) | the_bytes[0] # uint16
        self._dig_T2 = self._convert_uint16_to_int16((the_bytes[3] << 8) | the_bytes[2])  # int16
        self._dig_T3 = self._convert_uint16_to_int16((the_bytes[5] << 8) | the_bytes[4])  # int16
        self._dig_P1 = (the_bytes[7] << 8) | the_bytes[6]  # uint16
        self._dig_P2 = self._convert_uint16_to_int16((the_bytes[9] << 8) | the_bytes[8])  # int16
        self._dig_P3 = self._convert_uint16_to_int16((the_bytes[11] << 8) | the_bytes[10])  # int16
        self._dig_P4 = self._convert_uint16_to_int16((the_bytes[13] << 8) | the_bytes[12])  # int16
        self._dig_P5 = self._convert_uint16_to_int16((the_bytes[15] << 8) | the_bytes[14])  # int16
        self._dig_P6 = self._convert_uint16_to_int16((the_bytes[17] << 8) | the_bytes[16])  # int16
        self._dig_P7 = self._convert_uint16_to_int16((the_bytes[19] << 8) | the_bytes[18])  # int16
        self._dig_P8 = self._convert_uint16_to_int16((the_bytes[21] << 8) | the_bytes[20])  # int16
        self._dig_P9 = self._convert_uint16_to_int16((the_bytes[23] << 8) | the_bytes[22])  # int16
        self._dig_H1 = the_bytes[25]  # uint8

        the_bytes = self._read_bytes_from_register(self._i2c_address, _REG_CALIB26, 7)  # Second run _REG_CALIB26 to _REG_CALIB32
        self._dig_H2 = self._convert_uint16_to_int16((the_bytes[1] << 8) | the_bytes[0])  # int16
        self._dig_H3 = the_bytes[2]  # uint8
        self._dig_H4 = self._convert_uint16_to_int16((the_bytes[3] << 4) | (the_bytes[4] & 0x0F))  # int16
        self._dig_H5 = self._convert_uint16_to_int16((the_bytes[5] << 4) | ((the_bytes[4] >> 4) & 0x0F))  # int16
        self._dig_H6 = self._convert_uint8_to_int8(the_bytes[6])  # int8

        self._calibration_parameters_retrieved = True

    def print_calibration_parameters(self):
        """Print the calibration parameters."""
        print( "dig_T1=" + str(self._dig_T1) + " uint16")
        print( "dig_T2=" + str(self._dig_T2) + " int16")
        print( "dig_T3=" + str(self._dig_T3) + " int16")
        print( "dig_P1=" + str(self._dig_P1) + " uint16")
        print( "dig_P2=" + str(self._dig_P2) + " int16")
        print( "dig_P3=" + str(self._dig_P3) + " int16")
        print( "dig_P4=" + str(self._dig_P4) + " int16")
        print( "dig_P5=" + str(self._dig_P5) + " int16")
        print( "dig_P6=" + str(self._dig_P6) + " int16")
        print( "dig_P7=" + str(self._dig_P7) + " int16")
        print( "dig_P8=" + str(self._dig_P8) + " int16")
        print( "dig_P9=" + str(self._dig_P9) + " int16")
        print( "dig_H1=" + str(self._dig_H1) + " uint8")
        print( "dig_H2=" + str(self._dig_H2) + " int16")
        print( "dig_H3=" + str(self._dig_H3) + " uint8")
        print( "dig_H4=" + str(self._dig_H4) + " int16")
        print( "dig_H5=" + str(self._dig_H5) + " int16")
        print("dig_H6=" + str(self._dig_H6) + " int8")

    def get_ctrl_hum_reg(self):
        """Get the CTRL_HUM values."""
        the_bytes = self._read_bytes_from_register(self._i2c_address, _REG_CTRL_HUM)
        reg_val = the_bytes[0]
        osrs_h = reg_val & 0x07  # 3 bits
        return reg_val, osrs_h

    def set_ctrl_hum_reg(self, osrs_h = OSRS_SKIPPED):
        """Set the CTRL_HUM values."""
        reg_val = osrs_h & 0x07  # 3 bits
        self._write_bytes_to_register(self._i2c_address, _REG_CTRL_HUM, bytes([reg_val]))
        return None

    def get_status_reg(self):
        """Get the STATUS values."""
        the_bytes = self._read_bytes_from_register(self._i2c_address, _REG_STATUS)
        reg_val = the_bytes[0]
        im_update = reg_val & 0x01
        measuring = (reg_val >> 3) & 0x01
        return reg_val, im_update, measuring

    def get_ctrl_meas_reg(self):
        """Get the CTRL_MEAS values."""
        the_bytes = self._read_bytes_from_register(self._i2c_address, _REG_CTRL_MEAS)
        reg_val = the_bytes[0]
        mode = reg_val & 0x03 # 2 bits
        osrs_p = (reg_val >> 2) & 0x07 # 3 bits
        osrs_t = (reg_val >> 5) & 0x07 # 3 bits
        return reg_val, mode, osrs_p, osrs_t

    def set_ctrl_meas_reg(self, mode = MODE_SLEEP, osrs_p = OSRS_SKIPPED, osrs_t = OSRS_SKIPPED):
        """Set the CTRL_MEAS values."""
        reg_val = ((osrs_t & 0x07) << 5) | ((osrs_p & 0x07) << 2) | (mode & 0x03)
        self._write_bytes_to_register(self._i2c_address, _REG_CTRL_MEAS, bytes([reg_val]))
        return None

    def get_config_reg(self):
        """Get the CONFIG values."""
        the_bytes = self._read_bytes_from_register(self._i2c_address, _REG_CONFIG)
        reg_val = the_bytes[0]
        spi3w_en = reg_val & 0x01 # 1 bits
        filter = (reg_val >> 2) & 0x07 # 3 bits
        t_sb = (reg_val >> 5) & 0x07 # 3 bits
        return reg_val, spi3w_en, filter, t_sb

    def set_config_reg(self, filter = FILTER_COEF_OFF, t_sb = STANDBY_T_0_5_MS):
        """Set the CONFIG values."""
        reg_val = ((t_sb & 0x07) << 5) | ((filter & 0x07) << 2)
        self._write_bytes_to_register(self._i2c_address, _REG_CONFIG, bytes([reg_val]))
        return None

    def get_raw_sensor_values(self):
        """Get the raw sensor readings for Pressure, Temperature, and Humidity."""
        # Get the ADC values
        # _REG_PRESS_MSB to _REG_HUM_LSB
        the_bytes = self._read_bytes_from_register(self._i2c_address, _REG_PRESS_MSB, 8)

        # Table 29
        # _REG_PRESS_MSB - bits 19:12
        # _REG_PRESS_LSB - bits 11:4
        # _REG_PRESS_XLSB - bits 3:0
        # Resolution 16...20 bit
        raw_press_uint32 = (int(the_bytes[0]) << 12) | (int(the_bytes[1]) << 4) | (int(the_bytes[2]) >> 4)
        # If skipped then output will be 0x800000
        raw_press_int32 = None
        if raw_press_uint32 != 0x80000:  # Check validity
            raw_press_int32 = self._convert_uint32_to_int32(raw_press_uint32 & 0x000FFFFF)


        # Table 30
        # _REG_TEMP_MSB - bits 19:12
        # _REG_TEMP_LSB - bits 11:4
        # _REG_TEMP_XLSB - bits 3:0
        # Resolution 16...20 bit
        raw_temp_uint32 = (int(the_bytes[3]) << 12) | (int(the_bytes[4]) << 4) | (int(the_bytes[5]) >> 4)
        # If skipped then output will be 0x800000
        raw_temp_int32 = None
        if raw_temp_uint32 != 0x80000:  # Check validity
            raw_temp_int32 = self._convert_uint32_to_int32(raw_temp_uint32 & 0x000FFFFF)


        # Table 31
        # _REG_HUM_MSB - bits 15:8
        # _REG_HUM_LSB - bits 7:0

        raw_hum_uint32 = (int(the_bytes[6]) << 8) | int(the_bytes[7])
        # If skipped then output will be 0x8000
        raw_hum_int16 = None
        if raw_hum_uint32 != 0x8000:  # Check validity
            raw_hum_int16 = self._convert_uint16_to_int16(raw_hum_uint32 & 0xFFFF)

        return raw_press_int32, raw_temp_int32, raw_hum_int16

    def get_sensor_values_float(self):
        """Get the sensor readings as floats for Pressure (Pa), Temperature (DegC), and Humidity (%rH)."""
        raw_press_int32, raw_temp_int32, raw_hum_int16 = self.get_raw_sensor_values()
        # Then compensate
        if not self._calibration_parameters_retrieved:
            self.retrieve_calibration_parameters()

        temperature = None
        pressure = None
        humidity = None

        # Check for validity
        # Temperature first for compensation
        if raw_temp_int32:
            temperature = self._compensate_temperature_float(raw_temp_int32)

        if raw_press_int32:
            pressure = self._compensate_pressure_float(raw_press_int32)

        if raw_hum_int16:
            humidity = self._compensate_humidity_float(raw_hum_int16)

        return pressure, temperature, humidity

    def _compensate_temperature_float(self, raw_temp_int32):
        """Compensation of Temperature (degC) as float. See Appendix A of datasheet."""
        var1 = (raw_temp_int32 / 16384.0 - self._dig_T1 / 1024.0) * self._dig_T2
        var2 = (raw_temp_int32 / 131072.0 - self._dig_T1 / 8192.0) * (raw_temp_int32 / 131072.0 - self._dig_T1 / 8192.0) * self._dig_T3
        self._temperature_for_compensation = var1 + var2

        temperature = (var1 + var2) / 5120.0

        if temperature < -40.0:
            temperature = -40.0
        elif temperature > 85.0:
            temperature = 85.0

        return temperature

    def _compensate_pressure_float(self, raw_press_int32):
        """Compensation of Pressure (Pa) as float. See Appendix A of datasheet."""
        var1 = (self._temperature_for_compensation / 2.0) - 64000.0
        var2 = var1 * var1 * self._dig_P6 / 32768.0
        var2 = var2 + (var1 * self._dig_P5 * 2.0)
        var2 = (var2 / 4.0) + (self._dig_P4 * 65536.0)
        var3 = self._dig_P3 * var1 * var1 / 524288.0
        var1 = (var3 + (self._dig_P2 * var1)) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self._dig_P1

        pressure = 0.0

        if var1 > 0.0:
            pressure = 1048576.0 - raw_press_int32
            pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1
            var1 = self._dig_P9 * pressure * pressure / 2147483648.0
            var2 = pressure * self._dig_P8 / 32768.0
            pressure = pressure + (var1 + var2 + self._dig_P7) / 16.0

        if pressure < 30000.0:
            pressure = 30000.0
        elif pressure > 110000.0:
            pressure = 110000.0

        return pressure

    def _compensate_humidity_float(self, raw_hum_int16):
        """Compensation of Humidity (%rH) as float. See Appendix A of datasheet."""
        humidity = self._temperature_for_compensation - 76800.0
        humidity = (raw_hum_int16 - (self._dig_H4 * 64.0 + self._dig_H5 / 16384.0 * humidity)) * (self._dig_H2 / 65536.0 * (1.0 + self._dig_H6 / 67108864.0 * humidity * (1.0 + self._dig_H3 / 67108864.0 * humidity)))
        humidity = humidity * (1.0 - self._dig_H1 * humidity / 524288.0)

        if humidity > 100.0:
            humidity = 100.0
        elif humidity < 0.0:
            humidity = 0.0

        return humidity

    # Reading and Writing to Registers. I2C See page 32 of datasheet.
    def _read_bytes_from_register(self, i2c_address: int, register_address: int, num_bytes: int = 1) -> bytes:
        """Read a num_bytes bytes from a starting register.
        Returns a bytes object."""
        # return self._i2c.readfrom_mem(i2c_address, register_address, num_bytes) # machine.I2C
        return self._i2c.mem_read(num_bytes, i2c_address, register_address)  # pyb.I2C

    def _write_bytes_to_register(self, i2c_address: int, register_address: int, the_bytes: bytes):
        """Write the_bytes to a starting register.
        Returns None."""
        # Write a series of register_address and value pairs.
        for b in the_bytes:
            # self._i2c.writeto_mem(i2c_address, the_reg_address_with_flags, the_bytes) # machine.I2C
            self._i2c.mem_write(bytes([b]), i2c_address, register_address)  # pyb.I2C
            register_address = register_address + 1  # increment the register address

    def _convert_uint32_to_int32(self, value):
        """Convert uint32 to int32 by checking sign bit and bit-twiddling."""
        if value & 0x80000000:
            value = (-value ^ 0xFFFFFFFF) + 1

        return value

    def _convert_uint16_to_int16(self, value):
        """Convert uint16 to int16 by checking sign bit and bit-twiddling."""
        if value & 0x8000:
            value = (-value ^ 0xFFFF) + 1

        return value

    def _convert_uint8_to_int8(self, value):
        """Convert uint8 to int8 by checking sign bit and bit-twiddling."""
        if value & 0x80:
            value = (-value ^ 0xFF) + 1

        return value


    def run_example(self):
        """A simple example showing how to initialise and run the BME280."""
        self.retrieve_calibration_parameters()
        self.print_calibration_parameters()

        # Get Control Registers
        reg_val, osrs_h = self.get_ctrl_hum_reg()
        print("ctrl_hum_reg=" + str(reg_val))
        reg_val, im_update, measuring = self.get_status_reg()
        print("status_reg=" + str(reg_val))
        reg_val, mode, osrs_p, osrs_t = self.get_ctrl_meas_reg()
        print("ctrl_meas_reg=" + str(reg_val))
        reg_val, spi3w_en, filter, t_sb = self.get_config_reg()
        print("config_reg=" + str(reg_val))

        # Setup to take measurements
        self.set_ctrl_hum_reg(osrs_h=OSRS_OVERSAMPLE_X_1)
        self.set_ctrl_meas_reg(mode=MODE_NORMAL, osrs_p=OSRS_OVERSAMPLE_X_1, osrs_t=OSRS_OVERSAMPLE_X_1)
        self.set_config_reg(filter=FILTER_COEF_OFF, t_sb=STANDBY_T_1000_MS)

        # Get Control Registers
        reg_val, osrs_h = self.get_ctrl_hum_reg()
        print("ctrl_hum_reg=" + str(reg_val))
        reg_val, im_update, measuring = self.get_status_reg()
        print("status_reg=" + str(reg_val))
        reg_val, mode, osrs_p, osrs_t = self.get_ctrl_meas_reg()
        print("ctrl_meas_reg=" + str(reg_val))
        reg_val, spi3w_en, filter, t_sb = self.get_config_reg()
        print("config_reg=" + str(reg_val))

        raw_press_int32, raw_temp_int32, raw_hum_int16 = self.get_raw_sensor_values()
        print("raw_press_int32=" + str(raw_press_int32))
        print("raw_temp_int32=" + str(raw_temp_int32))
        print("raw_hum_int16=" + str(raw_hum_int16))

        while (not raw_press_int32) or (not raw_temp_int32) or (not raw_hum_int16):
            raw_press_int32, raw_temp_int32, raw_hum_int16 = self.get_raw_sensor_values()

        print("raw_press_int32=" + str(raw_press_int32))
        print("raw_temp_int32=" + str(raw_temp_int32))
        print("raw_hum_int16=" + str(raw_hum_int16))

        if raw_temp_int32:
            self._compensate_temperature_float(raw_temp_int32)
            print("_compensate_temperature_float=" + str(self._temperature_for_compensation))


def main():
    import pyb
    pyb.Pin.board.EN_3V3.off()
    pyb.delay(2000)

    pyb.Pin.board.EN_3V3.on()
    pyb.Pin('PULL_SCL', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X9/SCL pull-up
    pyb.Pin('PULL_SDA', pyb.Pin.OUT, value=1)  # enable 5.6kOhm X10/SDA pull-up
    # i2c = machine.I2C(1, freq=400000)  # machine.I2C
    i2c = pyb.I2C(1)  # pyb.I2C
    i2c.init(pyb.I2C.MASTER, baudrate=400000)  # pyb.I2C

    pyb.delay(500)

    sensor = BME280(i2c)

    sensor.soft_reset_device()

    pyb.delay(500)

    while not sensor.test_id():
        continue

    sensor.run_example()

    pyb.delay(100)

    pressure = 0.0
    temperature = 0.0
    humidity = 0.0

    awaiting_valid_measurements = True
    while awaiting_valid_measurements:

        measuring_flag = True
        while measuring_flag:
            reg_val, im_update, measuring = sensor.get_status_reg()
            if not measuring:
                measuring_flag = False

        pressure, temperature, humidity = sensor.get_sensor_values_float()

        if pressure and temperature and humidity:
            awaiting_valid_measurements = False

    print("pressure=" + str(pressure / 100.0) + " hPa")
    print("temperature=" + str(temperature) + " degC")
    print("humidity=" + str(humidity) + " %rH")


if __name__ == '__main__':
    main()