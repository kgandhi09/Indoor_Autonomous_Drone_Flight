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


def assign_values():

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

    #print ("Gx=%.2f" % Gx, '\u00b0' + "/s", "\tGy=%.2f" % Gy, '\u00b0' + "/s", "\tGz=%.2f" % Gz, '\u00b0' + "/s",
    #   	"\tAx=%.2f g" % Ax, "\tAy=%.2f g" % Ay, "\tAz=%.2f g" % Az)
    print('\n')
    print_fn_2()
    print("X Rotation = %.2f" % get_x_rotation(Ax, Ay, Az))
    print("Y Rotation = %.2f" % get_y_rotation(Ax, Ay, Az))
    # print('Gyro_X=' + str(gyro_x), 'Gyro_Y=' + str(gyro_y), 'Gyro_Z=' + str(gyro_z) )
    # print('Gx=%.2f' % Gx, 'Gy=%.2f' % Gy, 'Gz=%.2f' % Gz)
    # print('Acc_X=' + str(acc_x), 'Acc_Y=' + str(acc_y), 'Acc_Z=' + str(acc_z))
    # print('Ax=%.2f' % Ax, 'Ay=%.2f' % Ay, 'Az=%.2f' % Az)
    time.sleep(0.5)

def gyro_stabilize(Gx, Gy):

    i = 1

     if Gx > 5:
	vehicle.channels.overrides[1] -= i

     elif Gx < -2:
	vehicle.channels.overrides[1] += i

     elif Gy > 5:
	vehicle.channels.overrides[2] -= i

    elif Gy < -2:
	vehicle.channels.overrides[2] += i 

def takeoff_with_gyro():
    if vehicle.channels.overrides[3] < 1700:
	vehicle.channels.overrides[3] += amt

    if vehicle.channels.overrides[3] == 1700:
	vehicle.channels.overrides[3] = 1700



while True:

    assign_values()
    takeoff_with_gyro()
