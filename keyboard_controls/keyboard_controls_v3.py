from __future__ import print_function
from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil
import sys, os
from optparse import OptionParser
import Tkinter as tk
import math
import smbus
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

sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, baud=921600,  wait_ready=True)

vehicle.armed = True
time.sleep(0.5)

vehicle.channels.overrides[3] = 1040  # Throttle
vehicle.channels.overrides[2] = 1499  # pitch
vehicle.channels.overrides[1] = 1502  # roll


amt = 10
amt_2 = 30
m = 0

def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Adress, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Adress, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.write_byte_data(Device_Address, addr)
    low = bus.write_byte_data(Device_Adress, addr + 1)

    value = ((high << 8) | low)

    if (value > 32768):
	value -= 65536

    return value

def dist(x,y):
    return math.sqrt((x*x) + (y*y))

def get_x_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return math.degrees(radians)

def get_y_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)


bus = smbus.SMBus(1)
Device_Address = 0x68

MPU_Init()

print("Reading GYroscope and Accelerometer Data")

while True:
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    Ax = acc_x/16834.0
    Ay = acc_y/16834.0
    Az = acc_z/16834.0

    Gx = gyro_x/131.0
    Gy = gyro_y/131.0
    Gz = gyro_z/131.0	

def print_fn_1(num):
    print("\nThrottle = " + str(num) + "% - " + str(vehicle.channels.overrides[3]))
    print("Pitch value - " + str(vehicle.channels.overrides[1]))
    print('Roll value - '+ str(vehicle.channels.overrides[2]))

def print_fn_2():
    print("Throttle - " + str(vehicle.channels.overrides[3]))
    print('Pitch value - ' + str(vehicle.channels.overrides[1]))
    print('Roll value - ' + str(vehicle.channels.overrides[2]))

def key_press(event):
    if m == 0:
        if event.char == event.keysym: # ----------- standard-keys
            if event.keysym == 'k':
                vehicle.channels.overrides[3] = 1000
                vehicle.channels.overrides[2] = 1499  # pitch
                vehicle.channels.overrides[1] = 1500  # roll
                print("kill")
                print("\nThrottle value - " + str(vehicle.channels.overrides[3]))
                print('Pitch value - ' + str(vehicle.channels.overrides[1]))
                print('Roll value - ' +   str(vehicle.channels.overrides[2]))
            elif event.keysym == 'w' and vehicle.channels.overrides[3] < 1500:
                vehicle.channels.overrides[3] += amt
                print_fn_2()

            elif event.keysym == 's' and vehicle.channels.overrides[3] > 1040:
                vehicle.channels.overrides[3] -= amt
                print_fn_2()


        else :
            if event.keysym == 'Up' :
                vehicle.channels.overrides[2] -= amt_2
                print("\nForward")
                print_fn_2()
                global m
                m = 1
            
            elif event.keysym == 'Down' :
                vehicle.channels.overrides[2] += amt_2 
                print("\nBackward")
                print_fn_2()
                global m
                m = 1
            
            elif event.keysym == 'Left' :
                vehicle.channels.overrides[1] -= amt_2 
                print("\nLeft")
                print_fn_2()
                global m
                m = 1
                
            elif event.keysym == 'Right' :
                vehicle.channels.overrides[1] += amt_2 
                print("\nRight")
                print_fn_2()
                global m
                m = 1

#    else:  # -- non standard keys
        
        #if event.keysym == 'Up':
        #   vehicle.channels.overrides[2] -= amt  # pitch-control =  nose down (to go forward)
        #  print("forward, on throttle ", (int(vehicle.channels.overrides[3])))

        #elif event.keysym == 'Down':
        #    vehicle.channels.overrides[2] += amt  # pitch-control =  nose up (to go backword)
        #    print("backward, on throttle ", (int(vehicle.channels.overrides[3])))

        #elif event.keysym == 'Left':
        #    vehicle.channels.overrides[1] -= amt  # roll-control = left (move leftwards)
        #    print("left, on throttle ", (int(vehicle.channels.overrides[3])))

        #elif event.keysym == 'Right':
        #    vehicle.channels.overrides[1] += amt  # roll control = right (move leftwards)
        #    print("right, on throttle ", (int(vehicle.channels.overrides[3])))


def key_down(event):
    if m == 1:
        vehicle.channels.overrides[2] = 1499
        vehicle.channels.overrides[1] = 1502
        print('\nThrottle value - ' + str(vehicle.channels.overrides[3]))
        print('Pitch value - ' + str(vehicle.channels.overrides[1]))
        print('Roll value - ' +  str(vehicle.channels.overrides[2]))
        global m
        m = 0

def quit():
    global root
    root.quit()
 

# - Read the keyboard with tkinter
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind('<KeyPress>', key_press)
root.bind('<KeyRelease>', key_down)
#root.bind_all('<Key>', key)
tk.Button(root, text="Quit", command=root.destroy).pack()
root.mainloop()
