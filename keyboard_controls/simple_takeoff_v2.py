from __future__ import print_function
import smbus
import time
import math
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import sys, os
from optparse import OptionParser
import argparse
import threading
from pynput.keyboard import Key, Controller, Listener
import readchar
import pickle
import pigpio

# pass argumemts while runnimg the code 
# to run the code - 
# 1. sudo pigpiod (only one time for each boot)
# 2. python simple_takeoff_v2.py --connect /dev/ttyS0

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

vehicle.armed = True

def angle(num):
    return num*60

print("\nStoring Gyro Values for reference!")

flag = False
flag_2 = False

list_channel_1 = []
list_channel_2 = []

#----------- storing gyro axis values --------------------- #
list_x = []
list_y = []
#list_z = []
count1 = 0

vehicle.armed = True
#vehicle.send_calibrate_gyro  # --- send gyro calibration to mavlink

while True:

    count1 += 1
    vehicle.armed = True
    pitch = angle(vehicle.attitude.pitch) # >>> y-axis
    roll = angle(vehicle.attitude.roll)   # >>> x-axis
    #yaw = angle(vehicle.attitude.yaw)

    list_y.append(pitch)
    list_x.append(roll)
    #list_z.append(yaw)
    time.sleep(0.3)

    if count1 > 60:
	break

max_x = max(list_x)  #-- maximum gyro value on x axis in degrees to be in stable condition
min_x = min(list_x)  #-- minimum gyro value on x axis in degrees to be in stable condition

max_y = max(list_y)  #-- maximum gyro value on y axis in degrees to be in stable condition
min_y = min(list_y)  #-- minimum gyro value on y axis in degrees to be in stable condition

#max_z = max(list_z)
#min_z = min(list_z)
#------------------------------------------------------------- #

print('\n'+"Max X Value = " + str(max_x))
print("Min X Value = " + str(min_x))

print('\n'+"Max Y Value = " + str(max_y))
print("Min Y Value = " + str(min_y))

#------------------------------------------------------------- #

vehicle.armed = True  # Arming Vehicle
#vehicle.channels.overrides[3]= 1040 # --- Throttle

#------------------------------------------------------------- #

keyboard = Controller()
amt = 10
sf = 1.5

# Takeoff by incrementing and decrementing throttle value by 10 units by up and down key arrow press.
#param, key
#key = keyboard pressed
def takeoff(key):
    global flag_2
    vehicle.armed = True
    if not flag:
        if key == Key.up: #and vehicle.channels.overrides[3] < 2000:
            #vehicle.channels.overrides[3] += amt
	    #print('Up = '+ str(vehicle.channels.overrides[3]))
	    channel_list[2] += amt
	    ppm.update_channels(channel_list)

        elif key == Key.down: #and vehicle.channels.overrides[3] > 1040:
	    #vehicle.channels.overrides[3] -= amt
	    #print('Down = ' + str(vehicle.channels.overrides[3]))
	    channel_list[2] -= amt
	    ppm.update_channels(channel_list)

	elif key == Key.shift:
	    flag_2 = True
	    print('flag 2 True')

	elif key == Key.ctrl:
	    flag_2 = False
	    print('flag 2 false')

        elif key == Key.esc:
	    #vehicle.channels.overrides[3] = 1000
	    channel_list[2] = 1000

# ----------------------------------------------------------- #

# position at which drone will be stable
#param, Gx, Gy
#Gx - current(real time) X rotation gyro value
#Gy - current(real time) Y rotation gyro value
def stable_pos(Gx, Gy):
    result = False
    if min_x - sf < Gx < max_x + sf and min_y - sf < Gy < max_y + sf:
	result = True
    return result

# Counter valuses if drone is not in stable position
#param, Gx, Gy
#Gx - current(real time) X rotation gyro value
#Gy - current(real time) Y rotation gyro value
def stabilize(Gx, Gy):
    i = 1
    global channel_list
    if not flag_2:
        # below are the unstable conditions
	#Left motion
        if Gx > max_x + sf: #and 1000 <= vehicle.channels.overrides[1] <= 2000:
	    #vehicle.channels.overrides[1] -= i
	    #vehicle.channels.overrides[2] -= i
	    channel_list[0] -= i 
	    ppm.update_channels(channel_list)
	#Right motion
        if Gx < min_x - sf: #and 1000 <= vehicle.channels.overrides[1] <= 2000:
	    #ehicle.channels.overrides[1] += i
	    #vehicle.channels.overrides[2] += i
	    channel_list[0] += i
	    ppm.update_channels(channel_list)
	#Backward Motion
        if Gy > max_y + sf: #and 1000 <= vehicle.channels.overrides[2] <= 2000:
	    #vehicle.channels.overrides[2] += i
	    #vehicle.channels.overrides[1] -= i
	    channel_list[1] -= i
	    ppm.update_channels(channel_list)
	#Forward Motion
        if Gy < min_y - sf: #and 1000 <= vehicle.channels.overrides[2] <= 2000:
	    #vehicle.channels.overrides[2] -= i
	    #vehicle.channels.overrides[1] += i
	    channel_list[1] += i
	    ppm.update_channels(channel_list)
	'''
	if Gz > max_z + sf: #and 1000 <= vehicle.channels.overrides[2] <= 2000:
	    vehicle.channels.overrides[4] -= i
        elif Gz < min_z - sf: #and 1000 <= vehicle.channels.overrides[2] <= 2000:
	    vehicle.channels.overrides[4] += i
	'''

# If throttle in stable condition - continue throttle value
# If throttle not in stable condition - stabilize and again continue throttle if stabilize true
def gyro():
    global flag
    global t1
    global t2
    amt2 = 1

    #vehicle.send_calibrate_gyro  # --- send gyro calibration to mavlink
    while True:

    	new_pitch = angle(vehicle.attitude.pitch)
    	new_roll = angle(vehicle.attitude.roll)
	#new_yaw = angle(vehicle.attitude.yaw)
        if stable_pos(new_roll, new_pitch):
	    flag = False
	    #print('True')
	else: #1030 < vehicle.channels.overrides[3] < 2000:
	    flag = True
	    stabilize(new_roll, new_pitch)
	    #list_channel_1.append(vehicle.channels.overrides[1])
	    #list_channel_2.append(vehicle.channels.overrides[2])
	    #print('False')
    	time.sleep(0.3)

# ----------------------------------------------------------- #
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
	for i in range(len(widths)):
	    self._widths[i] -= 400
        self._update()

    def cancel(self):
        self.pi.wave_tx_stop()
        for i in self._wid:
            if i is not None:
                self.pi.wave_delete(i)
#------------------------------------------------------------- #

if __name__ == '__main__':

    import time 
    import pigpio
    
    pi = pigpio.pi()
    
    pi.wave_tx_stop()  # Start with a clean slate.
    ppm = X(pi, 6, frame_ms=20)
    updates = 0
    
    ch_1 = 1500 # pitch
    ch_2 = 1500 # roll
    ch_3 = 1000 # throttle
    ch_4 = 1500
    ch_5 = 1000
    ch_6 = 1000
    ch_7 = 1000
    ch_8 = 1000

    channel_list = [ch_1, ch_2, ch_3, ch_4, ch_5, ch_6, ch_7, ch_8]  # ppm signal list
   
    ppm.update_channels(channel_list)

    vehicle.armed = True
 
    t1 = threading.Thread(target = gyro)
    t2 = threading.Thread(target = takeoff(readchar.readkey()))

    t1.start()

    lis = Listener(on_press = takeoff)

    t2.start()
    lis.start()
    lis.join()
    
    time.sleep(2)

    ppm.cancel()

    pi.stop()
