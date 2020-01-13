# ANT - Cadence, Speed Sensor AND Heart Rate Monitor - Example
#
# Copyright (c) 2012, Gustav Tiger <gustav@tiger.name>
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

from __future__ import absolute_import, print_function

from ant.easy.node import Node
from ant.easy.channel import Channel
from ant.base.message import Message

from threading import Thread

import logging
import struct
import threading
import sys
import time

NETWORK_KEY= [0xb9, 0xa5, 0x21, 0xfb, 0xbd, 0x72, 0xc3, 0x45]
WHEEL_CIRCUMFERENCE = 2.105
# POWER_CURVE = [0.20, 45.92544, 0.0109137, 0.04466844] 
POWER_CURVE = [0.029, 26.53677, 0.000410018, 0.0282753488]

TYPE_SPEED = 123
TYPE_POWER = 11

class Monitor(Thread):

    def run(self):
        try:
            self.channel_speed.open()
            self.node.start()
        finally:
            self.node.stop()


    def __init__(self, type):
        Thread.__init__(self)
        self.speed = 0
        self.power = 0
        self.prev_val = None
        self.prev_time = None

        self.power_curve_coefs = POWER_CURVE

        node = Node()
        node.set_network_key(0x00, NETWORK_KEY)

        channel_speed = node.new_channel(Channel.Type.BIDIRECTIONAL_RECEIVE)

        channel_speed.on_broadcast_data = self.on_data_speed
        channel_speed.on_burst_data = self.on_data_speed

        if type == TYPE_SPEED:
            channel_speed.set_period(8118)
            channel_speed.set_search_timeout(30)
            channel_speed.set_rf_freq(57)
            channel_speed.set_id(0, TYPE_SPEED, 0)

        self.node = node
        self.channel_speed = channel_speed

    def power_curve(self, speed):
        if speed == 0:
            return 0
        return (self.power_curve_coefs[0] + self.power_curve_coefs[1]*speed +
                    self.power_curve_coefs[2]*speed*speed + self.power_curve_coefs[3]*speed*speed*speed)

    def on_data_speed(self, data):
        # self.cadence = str(data[3]*256 + data[2])
        new_val = data[7]*256 + data[6]
        new_time = time.time()
        if self.prev_time is not None and (new_val > self.prev_val + 3 or new_time > self.prev_time + 2):
            print(new_val)
            delay = (new_time - self.prev_time)
            self.speed = (new_val - self.prev_val) * WHEEL_CIRCUMFERENCE / delay
            self.prev_val = new_val
            self.prev_time = new_time
            self.power = self.power_curve(self.speed)
            self.display()

        if self.prev_val is None:
            self.prev_val = new_val
            self.prev_time = new_time

    def display(self):
        print("Speed: %.2fkm/h" % (self.speed * 3.6))
        print("Power: %.2fW" % self.power)


def main():
    monitor = Monitor(TYPE_SPEED)


    
if __name__ == "__main__":
    main()

