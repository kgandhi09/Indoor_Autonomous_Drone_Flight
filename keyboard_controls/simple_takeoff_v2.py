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

#vehicle.channels.overrides[3] = 1040 # --- Thhrottle
vehicle.channels.overrides[2] = 1499 # --- Pitch
vehicle.channels.overrides[1] = 1502 # --- Roll
#vehicle.channels.overrides[4] = 1500

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

max_x = max(list_x)  #-- maximum gyro value on x axis in degrees
min_x = min(list_x)  #-- minimum gyro value on x axis in degrees

max_y = max(list_y)  #-- maximum gyro value on y axis in degrees
min_y = min(list_y)  #-- minimum gyro value on y axis in degrees

#max_z = max(list_z)
#min_z = min(list_z)
#------------------------------------------------------------- #

print('\n'+"Max X Value = " + str(max_x))
print("Min X Value = " + str(min_x))

print('\n'+"Max Y Value = " + str(max_y))
print("Min Y Value = " + str(min_y))

#------------------------------------------------------------- #

vehicle.armed = True  # Arming Vehicle
vehicle.channels.overrides[3]= 1040 # --- Throttle

#------------------------------------------------------------- #

keyboard = Controller()
amt = 10
sf = 1.5

# Takeoff by incrementing and decrementing throttle value by 10 units by up and down key arrow press.
#param, key
#key = keyboard pressed
def takeoff(key):
    global flag_2
    if not flag:
        if key == Key.up and vehicle.channels.overrides[3] < 2000:
            vehicle.channels.overrides[3] += amt
	    #print('Up = '+ str(vehicle.channels.overrides[3]))

        elif key == Key.down and vehicle.channels.overrides[3] > 1040:
	    vehicle.channels.overrides[3] -= amt
	    #print('Down = ' + str(vehicle.channels.overrides[3]))

	elif key == Key.shift:
	    flag_2 = True
	    print('flag 2 True')

	elif key == Key.ctrl:
	    flag_2 = False
	    print('flag 2 false')

        elif key == Key.esc:
	    vehicle.channels.overrides[3] = 1000

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

# Counter valu es if drone is not in stable position
#param, Gx, Gy
#Gx - current(real time) X rotation gyro value
#Gy - current(real time) Y rotation gyro value
def stabilize(Gx, Gy):
    i = 8
    if not flag_2:
	#Left motion
        if Gx > max_x + sf: #and 1000 <= vehicle.channels.overrides[1] <= 2000:
	    vehicle.channels.overrides[1] -= i
	    vehicle.channels.overrides[2] -= i
	#Right motion
        elif Gx < min_x - sf: #and 1000 <= vehicle.channels.overrides[1] <= 2000:
	    vehicle.channels.overrides[1] += i
	    vehicle.channels.overrides[2] += i
	#Backward Motion
        if Gy > max_y + sf: #and 1000 <= vehicle.channels.overrides[2] <= 2000:
	    vehicle.channels.overrides[2] += i
	    vehicle.channels.overrides[1] -= i
	#Forward Motion
        elif Gy < min_y - sf: #and 1000 <= vehicle.channels.overrides[2] <= 2000:
	    vehicle.channels.overrides[2] -= i
	    vehicle.channels.overrides[1] += i
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
	elif 1030 < vehicle.channels.overrides[3] < 2000:
	    flag = True
	    stabilize(new_roll, new_pitch)
	    #list_channel_1.append(vehicle.channels.overrides[1])
	    #list_channel_2.append(vehicle.channels.overrides[2])
	    #print('False')
    	time.sleep(0.3)

# ----------------------------------------------------------- #

if __name__ == '__main__':

    t1 = threading.Thread(target = gyro)
    t2 = threading.Thread(target = takeoff(readchar.readkey()))

    t1.start()

    lis = Listener(on_press = takeoff)

    t2.start()
    lis.start()
    lis.join()
