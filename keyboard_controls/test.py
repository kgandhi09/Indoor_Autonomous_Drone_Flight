from __future__ import print_function
import smbus
import time 
import math
from dronekit import connect
from pymavlink import mavutil
import sys, os
from optparse import OptionParser
import argparse
from pynput.keyboard import Key, Controller, Listener
import readchar
import threading
import pickle
import RPi.GPIO as gpio

keyboard = Controller()


parser = argparse.ArgumentParser(
    description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect',
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
                    
parser.add_argument('--baudrate', 
                    help="Specify the baudrate of controller's serial port used for companion aircraft.")
                    
parser.add_argument('--aircraft', 
                    help="Specify the location to save the logs.")
                    
args = parser.parse_args()

    

connection_string = args.connect

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, baud=921600,  wait_ready=False)


list_x = []
count = 0

#vehicle.armed = True
time.sleep(0.5)

while True:
    #count += 1
    pitch = vehicle.attitude.pitch*60
    roll = vehicle.attitude.roll*60
    print('\n' + str(roll))
    print(pitch)
    #print(vehicle.groundspeed)
    #print(vehicle.ekf_ok)
    #time.sleep(0.3)
    #print(vehicle.attitude.roll*60, vehicle.attitude.pitch*60)
    break
    #if vehicle.on_message('HEARTBEAT'):
	#print(vehicle.attitude.roll*60)

#<<<<<<< HEAD
 #   time.sleep(0.2)
#==#=====
#    time.sleep(0.1)


# !/usr/bin/env python

# PPM.py
# 2016-02-18
# Public Domain

#import time
import pigpio


class X:

    GAP = 400
    WAVES = 3

    def __init__(self, pi, gpio, channels=8, frame_ms=27):
        self.pi = pi
        self.gpio = gpio

        if frame_ms < 5:
            frame_ms = 5
            channels = 2
        elif frame_ms > 100:

            frame_ms = 100

        self.frame_ms = frame_ms

        self._frame_us = int(frame_ms * 1000)
        self._frame_secs = frame_ms / 1000.0

        if channels < 1:
            channels = 1
        elif channels > (frame_ms // 2):
            channels = int(frame_ms // 2)

        self.channels = channels

        self._widths = [1000] * channels  # set each channel to minimum pulse width

        self._wid = [None] * self.WAVES
        self._next_wid = 0

        pi.write(gpio, pigpio.LOW)

        self._update_time = time.time()

    def _update(self):
        wf = []
        micros = 0
        for i in self._widths:
            wf.append(pigpio.pulse(0, 1 << self.gpio, self.GAP))
            wf.append(pigpio.pulse(1 << self.gpio, 0, i))
            micros += (i + self.GAP)
        # off for the remaining frame period
        wf.append(pigpio.pulse(1 << self.gpio,0, self._frame_us - micros))

        self.pi.wave_add_generic(wf)
        wid = self.pi.wave_create()
        self.pi.wave_send_using_mode(wid, pigpio.WAVE_MODE_REPEAT_SYNC)
        self._wid[self._next_wid] = wid

        self._next_wid += 1
        if self._next_wid >= self.WAVES:
            self._next_wid = 0

        remaining = self._update_time + self._frame_secs - time.time()
        if remaining > 0:
            time.sleep(remaining)
        self._update_time = time.time()

        wid = self._wid[self._next_wid]
        if wid is not None:
            self.pi.wave_delete(wid)
            self._wid[self._next_wid] = None

    def update_channel(self, channel, width):
        self._widths[channel] = width
        self._update()

    def update_channels(self, widths):
        self._widths[0:len(widths)] = widths[0:self.channels]
        self._update()

    def cancel(self):
        self.pi.wave_tx_stop()
        for i in self._wid:
            if i is not None:
                self.pi.wave_delete(i)


if __name__ == "__main__":

    import time
    #import PPM
    import pigpio

    pi = pigpio.pi()

    if not pi.connected:
	print('pi not connected!')
        exit(0)

    pi.wave_tx_stop()  # Start with a clean slate.
    print('hello!')
    ppm = X(pi, 6, frame_ms=20)
    print('world!')
    updates = 0
    start = time.time()
    #for chan in range(8):
     #   for pw in range(1000, 2000, 5):
      #      ppm.update_channel(chan, pw)
       #     updates += 1
	#    print(pw)
    end = time.time()
    secs = end - start

    ch_1 = 1100
    ch_2 = 1100
    ch_3 = 600
    ch_4 = 1100
    ch_5 = 600
    ch_6 = 600
    ch_7 = 600
    ch_8 = 600

    channel_list = [ch_1, ch_2, ch_3, ch_4, ch_5, ch_6, ch_7, ch_8]

    #ppm.update_channel(2, channel_list[2])
    #channel_list[2] += 50
    #ppm.update_channels(channel_list)

    #for i in range(200):
	#ppm.update_channels(channel_list)
	#channel_list[0] += 1
	#ppm.update_channels(channel_list)
        #print(channel_list)
    #print(ch_2)

    while True:
	vehicle.armed = True
	channel_list[2] += 1
        ppm.update_channels(channel_list)
   	print(channel_list)
	if channel_list[2] > 1600:
	    break
    time.sleep(2)

    ppm.cancel()

    pi.stop()

