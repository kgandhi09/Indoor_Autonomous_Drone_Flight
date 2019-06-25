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
vehicle = connect(connection_string, baud=921600,  wait_ready=True)


list_x = []
count = 0

vehicle.armed = True
time.sleep(0.5)

vehicle.send_calibrate_vehicle_level
print('Vehicle level')
time.sleep(1)
vehicle.send_calibrate_gyro
print('Gyro calibrated')
time.sleep(1)
vehicle.send_calibrate_accelerometer
print('Accel calibrated')
time.sleep(1)
vehicle.send_calibrate_magnetometer
print('Magnetometer calibrated')
time.sleep(1)

while True:
    #count += 1
    #pitch = vehicle.attitude.pitch*60
    #roll = vehicle.attitude.roll*60
    #print(roll)
    #print(pitch)
    #print(vehicle.groundspeed)
    #print(vehicle.ekf_ok)
    #time.sleep(0.3)

    print(vehicle.attitude.roll*60, vehicle.attitude.pitch*60)

    time.sleep(0.1)
