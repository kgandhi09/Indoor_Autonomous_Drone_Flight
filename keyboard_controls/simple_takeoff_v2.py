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
vehicle = connect(connection_string, baud=921600,  wait_ready=True)

vehicle.channels.overrides[3] = 1040 # --- Thhrottle
vehicle.channels.overrides[2] = 1499 # --- Pitch
vehicle.channels.overrides[1] = 1502 # --- Roll

def angle(num):
    return num*90

print("\nStoring Gyro Values for reference!")

flag = False

#----------- storing gyro axis values --------------------- #
list_x = []
list_y = []

count1 = 0

while True:

    count1 += 1

    pitch = angle(vehicle.attitude.pitch) # >>> y-axis
    roll = angle(vehicle.attitude.roll)   # >>> x-axis

    list_y.append(pitch)
    list_x.append(roll)

    time.sleep(0.5)

    if count1 > 60:
	break

max_x = max(list_x)  #-- maximum gyro value on x axis in degrees
min_x = min(list_x)  #-- minimum gyro value on x axis in degrees

max_y = max(list_y)  #-- maximum gyro value on y axis in degrees
min_y = min(list_y)  #-- minimum gyro value on y axis in degrees
#------------------------------------------------------------- #

print('\n'+"Max X Value = " + str(max_x))
print("Min X Value = " + str(min_x))

print('\n'+"Max Y Value = " + str(max_y))
print("Min Y Value = " + str(min_y))

#------------------------------------------------------------- #

keyboard = Controller()
amt = 10

def takeoff(key):

    if not flag:
        if key == Key.up and vehicle.channels.overrides[3] < 2000:
   	    vehicle.channels.overrides[3] += amt
	    #print('Up = '+ str(vehicle.channels.overrides[3]))

        elif key == Key.down and vehicle.channels.overrides[3] > 1040:
	    vehicle.channels.overrides[3] -= amt
	    #print('Down = ' + str(vehicle.channels.overrides[3]))

        elif key == Key.esc:
	    vehicle.channels.overrides[3] = 1000

# ----------------------------------------------------------- #

def stable_pos(Gx, Gy):
    result = False
    if min_x - 2 < Gx < max_x + 2 and min_y - 2 < Gy < max_y + 2:
	result = True
    return result

def stabilize(Gx, Gy):
    i = 1
    if Gx > max_x + 2:
	vehicle.channels.overrides[1] -= i
    if Gx < min_x - 2:
	vehicle.channels.overrides[1] += i
    if Gy > max_y + 2:
	vehicle.channels.overrides[2] -= i
    if Gy < min_y - 2:
	vehicle.channels.overrides[2] ++ i

def gyro():
    global flag
    global t1
    global t2
    amt2 = 1
    while True:

    	new_pitch = angle(vehicle.attitude.pitch)
    	new_roll = angle(vehicle.attitude.roll)

        if stable_pos(new_roll, new_pitch):
	    flag = False 
	    print('True')
	elif 1030 < vehicle.channels.overrides[3] < 2000:
	    flag = True
	    print('False')

   	time.sleep(1)

# ----------------------------------------------------------- #

def kill(self):
    self.killed = True

t1 = threading.Thread(target = gyro)
t2 = threading.Thread(target = takeoff(readchar.readkey()))

if __name__ == '__main__':

    t1.start()

    lis = Listener(on_press = takeoff)

    t2.start()
    lis.start()
    lis.join()
