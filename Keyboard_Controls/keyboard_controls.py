from __future__ import print_function
from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil
import sys, os
from optparse import OptionParser
import Tkinter as tk

# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(
    description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect',
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
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
vehicle = connect(connection_string, wait_ready=True)

print("Arming motors")
vehicle.armed = True
time.sleep(0.5)


# -- Key event function
def key(event):
    amt = 100
    vehicle.channels.overrides[3] = 1200  # Throttle
    vehicle.channels.overrides[2] = 1500  # pitch
    vehicle.channels.overrides[1] = 1500  # roll

    if event.char == event.keysym:  # -- standard keys
        if event.keysym == 'k':
            vehicle.channels.overrides[3] = 1000
            vehicle.channels.overrides[2] = 1000  # pitch
            vehicle.channels.overrides[1] = 1000  # roll
            print("kill")

        elif event.keysym == '1':
            vehicle.channels.overrides[3] = 1100
            print("Throttle = 10%")

        elif event.keysym == '2':
            vehicle.channels.overrides[3] = 1200
            print("Throttle = 20%")

        elif event.keysym == '3':
            vehicle.channels.overrides[3] = 1300
            print("Throttle = 30%")

        elif event.keysym == '4':
            vehicle.channels.overrides[3] = 1400
            print("Throttle = 40%")

        elif event.keysym == '5':
            vehicle.channels.overrides[3] = 1500
            print("Throttle = 50%")

        elif event.keysym == '6':
            vehicle.channels.overrides[3] = 1600
            print("Throttle = 60%")

        elif event.keysym == '7':
            vehicle.channels.overrides[3] = 1700
            print("Throttle = 70%")

        elif event.keysym == '8':
            vehicle.channels.overrides[3] = 1800
            print("Throttle = 80%")

        elif event.keysym == '9':
            vehicle.channels.overrides[3] = 1900
            print("Throttle = 90%")

        elif event.keysym == '0':
            vehicle.channels.overrides[3] = 2000
            print("Throttle = 100%")


    else:  # -- non standard keys
        if event.keysym == 'Up':
            vehicle.channels.overrides[2] -= amt  # pitch-control =  nose down (to go forward)
            vehicle.channels.overrides[3] = 1800
            print("forward")

        elif event.keysym == 'Down':
            vehicle.channels.overrides[2] += amt  # pitch-control =  nose up (to go backword)
            vehicle.channels.overrides[3] = 1800
            print("backward")

        elif event.keysym == 'Left':
            vehicle.channels.overrides[1] -= amt  # roll-control = left (move leftwards)
            vehicle.channels.overrides[3] = 1800
            print("left")

        elif event.keysym == 'Right':
            vehicle.channels.overrides[1] += amt  # roll control = right (move leftwards)
            vehicle.channels.overrides[3] = 1800
            print("right")


def quit():
    global root
    root.quit()


# - Read the keyboard with tkinter
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
tk.Button(root, text="Quit", command=root.destroy).pack()
root.mainloop()
