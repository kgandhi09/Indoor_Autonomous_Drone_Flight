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

def print_fn():
    print("Throttle - " + str(vehicle.channels.overrides[3]))
    print('Pitch value - ' + str(vehicle.channels.overrides[1]))
    print('Roll value - ' + str(vehicle.channels.overrides[2]))

vehicle.armed = True
time.sleep(0.5)

vehicle.channels.overrides[3] = 1040  # --- Throttle
vehicle.channels.overrides[2] = 1499  # --- Pitch
vehicle.channels.overrides[1] = 1502  # --- Roll

amt = 1

list_x = []
list_y = []

count = 0

def sf(num):
    if -1 < num < 0:
	return -(8*num)

    if 0 < num < 1:
	return (8*num)

    if num <= -1:
	return -((80*num)/100)

    if num >= 1:
    	return (80*num)/100

def gyro_balance(Gx, Gy):

    if min_x < Gx < max_x and min_y < Gy < max_y:
	vehicle.channels.overrides[1] = 1502 # ------ Roll
	vehicle.channels.overrides[2] = 1499 # ------ Pitch

    else:

        if Gx > max_x + 3:
    	    vehicle.channels.overrides[1] -= amt

        if Gx < min_x - 3:
   	    vehicle.channels.overrides[1] += amt

        if Gy > max_y + 3:
	    vehicle.channels.overrides[2] += amt

        if Gy < min_y - 3:
	    vehicle.channels.overrides[2] -= amt

    #print("X Axis = " + str(vehicle.channels.overrides[1]))
    #print("Y Axis = " + str(vehicle.channels.overrides[2]))
    #print(min_x - 4 < Gx < max_x + 4 and min_y - 4 < Gy < max_y + 4) # ----- stable 
    #print(Gx > max_x + 3)
    #print(Gx < min_x - 3)
    #print(Gy > max_y + 3)
    #print(Gy < min_y - 3)

print('\n'+"Calibrating gyro sensor")

def convert_to_angles(num):
    return num*90

while True:
    count += 1
    vehicle.channels.overrides[3] = 1040
    pitch = convert_to_angles(vehicle.attitude.pitch)
    roll = convert_to_angles(vehicle.attitude.roll)

    list_x.append(roll)
    list_y.append(pitch)

    print('\n')
    print("X rotation = " + str(roll))
    print("Y rotation = " + str(pitch))

    time.sleep(1)

    if count > 30:
	break

time.sleep(2)
print('\n'+"Calibration Done!" )
time.sleep(2)

max_x = max(list_x)
min_x = min(list_x)
max_y = max(list_y)
min_y = min(list_y)

print('\n'+"Max X Value = " + str(max_x))
print("Min X Value = " + str(min_x))
print('\n'+"Max Y Value = " + str(max_y))
print("Min Y Value = " + str(min_y))

time.sleep(2)
print('\n' + "Ready For TakeOff!")
time.sleep(2)
print('\n' + "Taking Off!" + '\n')
time.sleep(2)

count2 = 0

def takeoff():
     global count2
     m = 0
     while True:
	if m == 0:
     	    vehicle.channels.overrides[3] = 1040
	    time.sleep(1)
            vehicle.channels.overrides[3] = 1200
	    time.sleep(1)
            vehicle.channels.overrides[3] = 1300
	    time.sleep(1)
	    m = 1

	if m == 1:
	    vehicle.channels.overrides[3] = 1500
	    time.sleep(1)
            count2 += 1

	if count2 > 25:
	    m = 2

	if m == 2:
	    vehicle.channels.overrides[3] = 1400
	    time.sleep(1)
	    vehicle.channels.overrides[3] = 1300
	    time.sleep(1)
            vehicle.channels.overrides[3] = 1200
	    time.sleep(1)
	    vehicle.channels.overrides[3] = 1100
	    time.sleep(1)
	    vehicle.channels.overrides[3] = 1040
	    time.sleep(1)
            vehicle.channels.overrides[3] = 1000

def gyro_stabilize():
    while True:
        new_pitch = convert_to_angles(vehicle.attitude.pitch)
        new_roll = convert_to_angles(vehicle.attitude.roll)

        #print('\n'+"X rotation = " + str(new_roll))
        #print("Y rotation = " + str(new_pitch))
        #print(vehicle.channels.overrides[3])

        gyro_balance(new_roll, new_pitch)

        time.sleep(1)

if __name__ == "__main__":
    t1 = threading.Thread(target=takeoff)
    t2 = threading.Thread(target=gyro_stabilize)

    t1.start()
    t2.start()
