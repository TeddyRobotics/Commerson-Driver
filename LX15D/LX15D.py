#!/usr/bin/env python
# encoding: utf-8

"""API for LX15D smart servo motors"""

# Copyright (c) 2019 Teddy Robotics LLC
# Authors: Carlos Fernandez and Walter Berreta
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

import serial
import time
from serial.serialutil import Timeout

SERVO_ID_ALL = 0xfe

EEPROM = {
        'SERVO_MOVE_TIME_WRITE': 1,
        'SERVO_MOVE_TIME_READ': 2,
        'SERVO_MOVE_TIME_WAIT_WRITE': 7,
        'SERVO_MOVE_TIME_WAIT_READ': 8,
        'SERVO_MOVE_START': 11,
        'SERVO_MOVE_STOP': 12,
        'SERVO_ID_WRITE': 13,
        'SERVO_ID_READ': 14,
        'SERVO_ANGLE_OFFSET_ADJUST': 17,
        'SERVO_ANGLE_OFFSET_WRITE': 18,
        'SERVO_ANGLE_OFFSET_READ': 19,
        'SERVO_ANGLE_LIMIT_WRITE': 20,
        'SERVO_ANGLE_LIMIT_READ': 21,
        'SERVO_VIN_LIMIT_WRITE': 22,
        'SERVO_VIN_LIMIT_READ': 23,
        'SERVO_TEMP_MAX_LIMIT_WRITE': 24,
        'SERVO_TEMP_MAX_LIMIT_READ': 25,
        'SERVO_TEMP_READ': 26,
        'SERVO_VIN_READ': 27,
        'SERVO_POS_READ': 28,
        'SERVO_OR_MOTOR_MODE_WRITE': 29,
        'SERVO_OR_MOTOR_MODE_READ': 30,
        'SERVO_LOAD_OR_UNLOAD_WRITE': 31,
        'SERVO_LOAD_OR_UNLOAD_READ': 32,
        'SERVO_LED_WRITE': 33,
        'SERVO_LED_READ': 34,
        'SERVO_LED_ERROR_WRITE': 35,
        'SERVO_LED_ERROR_READ': 36
         }

class LX15D:
    def __init__(self, serial, time_out = 1):
        self._serial = serial
        self._timeout = time_out

    #Commands-------------------
    def _command(self, servo_id, instruction, *params):
        """
        command packet format
        Header-----ID number--Data Length--Command-Parameter-------Checksum
        0x55 0x55-- ID----------  Length-----Cmd--Prm 1... Prm N---Checksum

        Length: equal to the length of the data that
        is to be sent(including its own one byte). That is,
        the length of the data plus 3 is equal to the
        length of command packet, from the header to the checksum.
        """
        length = 3 + len(params)
        #print('length', length)
        """
        checksum calculation:
        checksum = ~(ID + length+instruction+parms) if the numbers in the brackets
        are calculated and exceeded 255, then it takes the lowest one byte, "~"
        means Negation
        """
        checksum = 255 - ((servo_id + length + instruction + sum(params))% 256)
        #print('checksum', checksum)
        packet = [0x55, 0x55, servo_id, length, instruction, *params, checksum]
        #print('packet', packet)
        self._serial.write(bytearray(packet))
        #print('Sending packet', packet)

    #ID--------------------------------
    def set_id(self, old_id, new_id):
        self._command(old_id, EEPROM['SERVO_ID_WRITE'], new_id )

    def get_servo_id(self, servo_id=SERVO_ID_ALL):
        self._command(servo_id, EEPROM['SERVO_ID_READ'])
        response = self._wait_for_response(servo_id)
        #print('ID:', response[5])

    def _wait_for_response(self, servo_id):

        def read(size=1):
            data = self._serial.read(size)
            if len(data) != size:
                raise TimeoutError()
            return data

        while True:
            data = []
            data += read(1)
            if data[-1] != 0x55:
                continue
            data += read(1)
            if data[-1] != 0x55:
                continue
            data += read(3)
            sid = data[2]
            length = data[3]
            cmd = data[4]
            if length > 7:
                print('invalid packet')
                continue
            data += read(length-3) if length > 3 else []
            params = data[5:]
            data += read(1)
            checksum = data[-1]
            if 255-(sid + length + cmd + sum(params)) % 256 != checksum:
                print('Invalid checksum for packet %s', list(data))
                continue
            #print(data)
            return data

    #MOVE COMMANDS---------------------
    def move(self, servo_id, target_position, angular_velocity):
        #print(target_position, velocity)
        initial_angle = int(self.get_position(servo_id))
        #print("angle ",initial_angle)
        #print(initial_angle)
        time_const = abs((initial_angle-target_position)/ angular_velocity)
        angle_conversion = target_position/0.24
        #print("Moving to angle: ", target_position)
        #motor has 0-1000 steps
        position = self.boundaries(0, 1000, angle_conversion)
        #Time can only be from 0 - 30000ms
        time_const = self.boundaries(0, 30000, time_const)
        print("Time for destination: ", time_const/1000)#Conversion to seconds
        self._command(
                    servo_id, EEPROM['SERVO_MOVE_TIME_WRITE'],
                    self.lower_byte(position), self.higher_byte(position),
                    self.lower_byte(time_const), self.higher_byte(time_const),
                     )
        #return time_const/1000 #Converted to seconds

    def raw_move(self, servo_id, position, time):
        self._command(
                    servo_id, EEPROM['SERVO_MOVE_TIME_WRITE'],
                    self.lower_byte(position), self.higher_byte(position),
                    self.lower_byte(time), self.higher_byte(time),
                     )

    def get_prepared_move(self, servo_id):
        self._command(servo_id, EEPROM['SERVO_MOVE_TIME_WAIT_READ'])
        response = self._wait_for_response(servo_id)
        return response
        #print(response)

    """Note that angular velocity equals to degrees/second, it ranges from 0.01 to ~0.2"""
    """0.18sec/60degree(6v)"""
    """0.16sec/60degree(7.4v)"""
    def move_prepare(self, servo_id, target_position, angular_velocity):
        #print(target_position, velocity)
        initial_angle = int(self.get_position(servo_id))
        #print(initial_angle)
        time_const = abs((initial_angle-target_position)/ angular_velocity)
        angle_conversion = target_position/0.24
        #print("Moving to angle: ", target_position)
        #motor has 0-1000 steps
        position = self.boundaries(0, 1000, angle_conversion)
        #print("position", position)
        #Time can only be from 0 - 30000ms
        time_const = self.boundaries(0, 30000, time_const)
        print("Time for target_position: ", time_const/1000)#Conversion to seconds
        self._command(
            servo_id, EEPROM['SERVO_MOVE_TIME_WAIT_WRITE'],
            self.lower_byte(position), self.higher_byte(position),
            self.lower_byte(time_const), self.higher_byte(time_const),
            )
        return time_const/1000 #Converted to seconds

    def move_start(self, servo_id=None):
        if servo_id is None :
            servo_id = SERVO_ID_ALL
            self._command(servo_id, EEPROM['SERVO_MOVE_START'])
        else:
            for x in servo_id:
                self._command(x, EEPROM['SERVO_MOVE_START'])


    def move_stop(self, servo_id=SERVO_ID_ALL):
        self._command(servo_id,EEPROM['SERVO_MOVE_STOP'])

    def get_position(self, servo_id):
        self._command(servo_id, EEPROM['SERVO_POS_READ'])
        response = self._wait_for_response(servo_id)
        #print("response: ", response)
        low = response[-3]
        high = response[-2]
        position = int(low) + int(high)*256
        position = round(position*0.24, 3)
        print(position)
        return position

    #TOGGLE MOTOR POWER----------------
    def enable_motor(self, id):
        self._command(id, EEPROM['SERVO_LOAD_OR_UNLOAD_WRITE'], 1)

    def disable_motor(self, id):
        self._command(id, EEPROM['SERVO_LOAD_OR_UNLOAD_WRITE'], 0)

    #LED TOGGLE------------------------
    def toggle_led_on(self):
        self._command(self._id, EEPROM['SERVO_LED_WRITE'], 0)

    def toggle_led_off(self):
        self._command(self._id, EEPROM['SERVO_LED_WRITE'], 1)

    #SPEED COMMANDS--------------------
    def get_speed(self, servo_id):
        self._command(servo_id, EEPROM['SERVO_OR_MOTOR_MODE_READ'])
        response = self._wait_for_response(servo_id)
        param_1 = response[5]#servo mode
        param_2 = response[6]
        param_3 = response[7]#lower byte
        param_4 = response[8]#higher byte
        speed_value = int(param_3) + int(param_4)*256
        print(speed_value)

    def set_speed(self, servo_id, speed = 0):
        speed = self.boundaries(-1000, 1000, speed)
        if speed < 0:
            speed += 65536
        self._command(servo_id, EEPROM['SERVO_OR_MOTOR_MODE_WRITE'], 1, 0, self.lower_byte(speed), self.higher_byte(speed))


    #Temperature-----------------------
    def get_temp(self, servo_id):
        self._command(servo_id, EEPROM['SERVO_TEMP_READ'])
        temperature = self._wait_for_response(servo_id)
        print("Servo ID: ",servo_id, " Temperature: ", temperature[5])
        return temperature[5]


    def set_max_temp_limit(self, max_temp):
        max_temp = boundaries(50, 100, max_temp)
        self._command(self._id, EEPROM['SERVO_TEMP_MAX_LIMIT_WRITE'], max_temp)

    def get_max_temp_limit(self, max_temp):
        max_temp = self._query(self._id, EEPROM['SERVO_TEMP_MAX_LIMIT_READ'], max_temp)

    #LED------------------------------
    def motor_led(self, state):
        self._command(self._id, EEPROM['SERVO_ID_WRITE'], state)

    def get_led_errors(self, servo_id):
        self._command(servo_id, EEPROM['SERVO_LED_ERROR_READ'])
        led_error = self._wait_for_response(servo_id)
        errors = {
                0:'No alarm',
                1:'Over temperature',
                2:'Over voltage',
                3:'Over temperature and over voltage',
                4:'Locked-rotor',
                5:'Over temperature and stalled',
                6:'Over voltage and stalled',
                7:'Over temperature , over voltage and stalled'
                  }
        print(led_error)
        if led_error[3] not in errors:
            raise Exception('Unknown type: {}'.format(led_error))
        result = errors[led_error[3]]
        print(result)

    #Load-----------------------------
    def get_load(self, servo_id):
        self._command(servo_id, EEPROM['SERVO_LOAD_OR_UNLOAD_READ'])
        servo_load = self._wait_for_response(servo_id)
        if servo_load[5] == 1:
            print('Torque is on:', servo_load[5])
        else:
            print('Torque is off:', servo_load[5])

    #BYTE HANDLINDS--------------------
    def lower_byte(self, value):
        return int(value)%256

    def higher_byte(self, value):
        return int(value/256)%256

    def word(self, low, high):
        return int(low) + int(high)*256

    def boundaries(self, min_val, max_val, input_val):
        return min(max_val, max(min_val, input_val))
