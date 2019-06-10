from __future__ import print_function
import smbus
import time 
import math
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import sys, os
from optparse import OptionParser
import argparse

# some MPU6050 Registers and their Address
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

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

vehicle.armed = True
time.sleep(0.5)

vehicle.channels.overrides[3] = 1100  #------ Throttle
vehicle.channels.overrides[2] = 1499  #------ Pitch
vehicle.channels.overrides[1] = 1502  #------ Roll

amt = 50
amt_2 = 30
m = 0

def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if (value > 32768):
        value = value - 65536
    return value

def dist(x,y):
   return math.sqrt((x*x) + (y*y))

def get_x_rotation(x,y,z):
   radians = math.atan2(x, dist(y,z))
   return -math.degrees(radians)

def get_y_rotation(x,y,z):
   radians = math.atan2(y, dist(x,z))
   return -math.degrees(radians)

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address

MPU_Init()

def print_fn_1(num):
    print("\nThrottle = " + str(num) + "% - " + str(vehicle.channels.overrides[3]))
    print("Pitch value - " + str(vehicle.channels.overrides[1]))
    print('Roll value - '+ str(vehicle.channels.overrides[2]))

def print_fn_2():
    print("Throttle - " + str(vehicle.channels.overrides[3]))
    print('Pitch value - ' + str(vehicle.channels.overrides[1]))
    print('Roll value - ' + str(vehicle.channels.overrides[2]))


Dx = 0
Dy = 0

list_x = []
list_y = []

count = 0

def assign_gyro_values():

    global Dx
    global Dy

    # Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    # Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0

    Dx = get_x_rotation(Ax, Ay, Az)
    Dy = get_y_rotation(Ax, Ay, Az)

    list_x.append(Dx)
    list_y.append(Dy)

    print('\n')
    print("X Rotation = %.2f" % Dx)
    print("Y Rotation = %.2f" % Dy)

def sensitivity_factor(num):
    if -1 < num < 0:
	return -(5*num)

    if 0 < num < 1:
	return (5*num)

    if num <= -1:
	return -((65*num)/100)

    if num >= 1:
    	return (65*num)/100

def gyro_balance(Gx, Gy):

    global x
    global y

    if min_x < Gx < max_x and min_y < Gy < max_y:
	vehicle.channels.overrides[1] = 1500
	vehicle.channels.overrides[2] = 1500

    else:

        if Gx > max_x + sensitivity_factor(max_x):
    	    vehicle.channels.overrides[1] = 1470

        if Gx < min_x - sensitivity_factor(min_x):
	    vehicle.channels.overrides[1] = 1530

        if Gy > max_y + sensitivity_factor(max_y):
	    vehicle.channels.overrides[2] = 1470

        if Gy < min_y - sensitivity_factor(min_y):
	    vehicle.channels.overrides[3] = 1530

    print("X Axis = " + str(x))
    print("Y Axis = " + str(y))
    #print(Gx > max_x + sf(max_x)) # ---- pos x axis
    #print(Gx < min_x - sf(min_x)) # ---- neg x axis
    #print(Gy > max_x + sf(max_y)) # ---- pos y axis
    #print(Gy < min_x - sf(min_y)) # ---- neg y axis


print("Callibrating external gyro sensor")

sleep(1)

while True:

    count += 1
    assign_gyro_values()

    max_x = round(max(list_x), 2)
    min_x = round(min(list_x), 2)
    max_y = round(max(list_y), 2)
    min_y = round(min(list_y), 2)

    if count > 1000:
	break

sleep(1)

print("Calibrate x and y min_max values - ")
print("\n")
print("Max X Value = " + str(max_x))
print("Min X Value = " + str(min_x))
print("Max Y Value = " + str(max_y))
print("Min Y Value = " + str(min_y))
print("\n")
print("Initializing gyro balance for drone")

sleep(4)

while True:

    assign_gyro_values()
    gyro_balance(Dx, Dy)
