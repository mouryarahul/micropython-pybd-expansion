#! /usr/bin/env python
#
# MicroPython Driver for PowerModule control onboard the PYBD Expansion Board.
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
"""MicroPython Driver for PowerModule control onboard the PYBD Expansion Board."""

import pyb

# Pin Names
_NM3EN_PINSTR = 'Y4'
_5VAEN_PINSTR = 'X1'
_5VBEN_PINSTR = 'X2'
_3V3AEN_PINSTR = 'X11'
_3V3BEN_PINSTR = 'X12'
_VBATT_PINSTR = 'Y12'


class PowerModule:
    """PowerModule control."""

    #def __init__(self):


    def __call__(self):
        return self

    def get_vbatt_reading(self):
        """Read VBatt level on ADC."""
        vbatt_adc = pyb.ADC(pyb.Pin(_VBATT_PINSTR, pyb.Pin.IN))
        vbatt_val = vbatt_adc.read()  # between 0 and 4095.
        # Convert to volts
        # potential divider of 100K and 390K.
        vbatt_volts = 4.9 * (vbatt_val / 4096.0) * 3.3
        return vbatt_volts

    def enable_nm3(self):
        """Enable power to the NM3."""
        # Set Pin as input so it is pulled high.
        pyb.Pin(_NM3EN_PINSTR, pyb.Pin.IN)

    def disable_nm3(self):
        """Disable power to the NM3."""
        # Set Pin as output and drive low.
        pyb.Pin(_NM3EN_PINSTR, pyb.Pin.OUT, value=0)

    def enable_5v_a(self):
        """Enable power to the 5V A."""
        # Set Pin as output and drive high.
        pyb.Pin(_5VAEN_PINSTR, pyb.Pin.OUT, value=1)

    def disable_5v_a(self):
        """Disable power to the 5V A."""
        # Set Pin as input so it is pulled low.
        pyb.Pin(_5VAEN_PINSTR, pyb.Pin.IN)

    def enable_5v_b(self):
        """Enable power to the 5V B."""
        # Set Pin as output and drive high.
        pyb.Pin(_5VBEN_PINSTR, pyb.Pin.OUT, value=1)

    def disable_5v_b(self):
        """Disable power to the 5V B."""
        # Set Pin as input so it is pulled low.
        pyb.Pin(_5VBEN_PINSTR, pyb.Pin.IN)

    def enable_3v3_a(self):
        """Enable power to the 3V3 A. Note this is supplied from the PYBD 3v3 Regulator."""
        # Set Pin as output and drive high.
        pyb.Pin(_3V3AEN_PINSTR, pyb.Pin.OUT, value=1)

    def disable_3v3_a(self):
        """Disable power to the 3V3 A."""
        # Set Pin as input so it is pulled low.
        pyb.Pin(_3V3AEN_PINSTR, pyb.Pin.IN)

    def enable_3v3_b(self):
        """Enable power to the 3V3 B. Note this is supplied from the PYBD 3v3 Regulator."""
        # Set Pin as output and drive high.
        pyb.Pin(_3V3BEN_PINSTR, pyb.Pin.OUT, value=1)

    def disable_3v3_b(self):
        """Disable power to the 3V3 B."""
        # Set Pin as input so it is pulled low.
        pyb.Pin(_3V3BEN_PINSTR, pyb.Pin.IN)



def main():
    import pyb
    # Cycle Power
    pyb.Pin.board.EN_3V3.off()
    pyb.delay(1000)

    pyb.Pin.board.EN_3V3.on()
    pyb.delay(500)

    # Create device
    power_module = PowerModule()

    while True:

        # Measure VBatt
        vbatt_volts = power_module.get_vbatt_reading()
        print("vbatt_volts=" + str(vbatt_volts) + "V")

        print("Disable Outputs")

        # Disable all Outputs
        power_module.disable_nm3()
        power_module.disable_3v3_a()
        power_module.disable_3v3_b()
        power_module.disable_5v_a()
        power_module.disable_5v_b()

        pyb.delay(5000)

        print("Enable Outputs")

        # Enable all Outputs
        power_module.enable_nm3()
        power_module.enable_3v3_a()
        power_module.enable_3v3_b()
        power_module.enable_5v_a()
        power_module.enable_5v_b()

        pyb.delay(5000)


if __name__ == '__main__':
    main()
