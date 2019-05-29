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

vehicle.channels.overrides[3] = 1200  # Throttle
vehicle.channels.overrides[2] = 1500  # pitch
vehicle.channels.overrides[1] = 1500  # roll


amt = 100
amt_2 = 5
m = 0
                
def key_press(event):
    if m == 0:
        if event.char == event.keysym: # ----------- standard-keys
            if event.keysym == 'k':
                vehicle.channels.overrides[3] = 1000
                vehicle.channels.overrides[2] = 1000  # pitch
                vehicle.channels.overrides[1] = 1000  # roll
                print("kill")
                print("\nThrottle value - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))
                

            elif event.keysym == '1':
                vehicle.channels.overrides[3] = 1100
                print("\nThrottle = 10% - ", int(vehicle.channels.overrides[3]))
                print("Pitch value - ", int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))
                

            elif event.keysym == '2':
                vehicle.channels.overrides[3] = 1200
                print("\nThrottle = 20% - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))
                

            elif event.keysym == '3':  
                vehicle.channels.overrides[3] = 1300
                print("\nThrottle = 30% - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))

            elif event.keysym == '4':
                vehicle.channels.overrides[3] = 1400
                print("\nThrottle = 40% - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))

            elif event.keysym == '5':
                vehicle.channels.overrides[3] = 1500
                print("\nThrottle = 50% - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))

            elif event.keysym == '6':
                vehicle.channels.overrides[3] = 1600
                print("\nThrottle = 60% - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))

            elif event.keysym == '7':
                vehicle.channels.overrides[3] = 1700
                print("\nThrottle = 70% - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))

            elif event.keysym == '8':
                vehicle.channels.overrides[3] = 1800
                print("\nThrottle = 80% - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))

            elif event.keysym == '9':
                vehicle.channels.overrides[3] = 1900
                print("\nThrottle = 90% - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))

            elif event.keysym == '0':
                vehicle.channels.overrides[3] = 2000
                print("\nThrottle = 100% - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))

        else :
            if event.keysym == 'Up' :
                vehicle.channels.overrides[2] -= amt_2
                print("\nForward")
                print("Throttle - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))
                global m
                m = 1
            
            elif event.keysym == 'Down' :
                vehicle.channels.overrides[2] += amt_2 
                print("\nBackward")
                print("Throttle - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))
                global m
                m = 1
            
            elif event.keysym == 'Left' :
                vehicle.channels.overrides[1] -= amt_2 
                print("\Left")
                print("Throttle - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))
                global m
                m = 1
                
            elif event.keysym == 'Right' :
                vehicle.channels.overrides[1] += amt_2 
                print("\nRight")
                print("Throttle - ", int(vehicle.channels.overrides[3]))
                print('Pitch value - ', int(vehicle.channels.overrides[1]))
                print('Roll value - ',  int(vehicle.channels.overrides[2]))
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
        vehicle.channels.overrides[1] = 1500
        vehicle.channels.overrides[2] = 1500
        print('\nThrottle value - ', int(vehicle.channels.overrides[3]))
        print('Pitch value - ', int(vehicle.channels.overrides[1]))
        print('Roll value - ',  int(vehicle.channels.overrides[2]))
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
