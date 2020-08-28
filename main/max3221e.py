#! /usr/bin/env python
#
# MicroPython Driver for MAX3221E onboard the PYBD Expansion Board.
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
"""MicroPython Driver for MAX3221E onboard the PYBD Expansion Board.
Wiring currently ties the !EN pin to ground ensuring the Receiver is always enabled."""

class MAX3221E:
    """MAX3221E Device."""

    def __init__(self, forceon_not_off_pin=None):
        """Init. forceon_not_off_pin is expected to have on() and off() functions."""
        self._forceon_not_off_pin = forceon_not_off_pin


    def __call__(self):
        return self

    def tx_force_on(self):
        """Force On the Tx Driver."""
        if self._forceon_not_off_pin:
            self._forceon_not_off_pin.on()

    def tx_force_off(self):
        """Force Off the Tx Driver."""
        if self._forceon_not_off_pin:
            self._forceon_not_off_pin.off()


def main():
    import pyb
    # Cycle Power
    pyb.Pin.board.EN_3V3.off()
    pyb.delay(1000)

    pyb.Pin.board.EN_3V3.on()
    pyb.delay(500)

    # Create device
    pyb.Pin('Y5', pyb.Pin.OUT, value=0)  # enable Y5 Pin as output
    max3221e = MAX3221E(pyb.Pin.board.Y5)
    uart = pyb.UART(1, 9600, bits=8, parity=None, stop=1, timeout=1000)

    # Power up Tx Driver
    max3221e.tx_force_on()

    cmd_string = '$?'
    cmd_bytes = cmd_string.encode('utf-8')
    uart.write(cmd_bytes)

    # Power down Tx Driver
    max3221e.tx_force_off()

    # Receive and process
    resp_bytes = uart.read(20)
    if resp_bytes:
        print(resp_bytes.decode('utf-8'))

    print("====Ends====")


if __name__ == '__main__':
    main()
