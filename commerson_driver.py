#!/usr/bin/env python
# encoding: utf-8

"""Commerson driver that allows communication with the LX15D motors"""

# Copyright (c) 2019 Teddy Robotics LLC
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from LX15D.LX15D import LX15D as lx
import serial

class CommersonDriver:
    def __init__(self):
        serial_port = '/dev/ttyAMA0'
        port = serial.Serial(port=serial_port,
                             baudrate=115200,
                             parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE,
                             bytesize=serial.EIGHTBITS,
                             timeout=1)
        self._servo = lx(port)
        
    def move_motor(self, motor_id, angle, speed):
        self._servo.move(motor_id, angle, speed)
