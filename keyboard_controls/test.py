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

keyboard = Controller()

'''
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
'''
'''
start = time.time()

def no_of_args(func):
    return str(func.func_code.co_varnames)

msg = vehicle.message_factory.hil_gps_encode(
    start,
    3,
    9.0,
    9.0,
    2,
    65535,
    65535,
    65535,
    0,
    0,
    0,
    65535,
    255,
    )

# print(msg)
#vehicle.send_mavlink(msg)

def test(self, attr_name, value):
    attr_name = 'attitude'
    print("got the data!")

#vehicle.add_message_listener('HIL_GPS', test)
#time.sleep(5)

#@vehicle.on_message('HEARTBEAT')
#def my_method(self, name, msg):
#    name = 'HEARTBEAT'
#    print('got the heartbeat')
'''

