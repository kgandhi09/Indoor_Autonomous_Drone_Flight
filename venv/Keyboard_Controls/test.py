from __future__ import print_function

import time

import sys, os
from optparse import OptionParser
import Tkinter as tk


const_3 = 1200  # Throttle
const_2 = 1500  # pitch
const_1 = 1500  # roll

amt = 100
amt_2 = 30

i = 0
m = 0


def print_fn_1(num):
    print("\nThrottle = " + str(num) + "% - " + str(const_3))
    print("Pitch value - " + str(const_1))
    print('Roll value - ' + str(const_2))


def print_fn_2():
    print("\nThrottle - " + str(const_3))
    print('Pitch value - ' + str(const_1))
    print('Roll value - ' + str(const_2))


def key_press(event):
    global const_3
    global const_2
    global const_1


    if m == 0:
        if event.char == event.keysym:  # ----------- standard-keys
            if event.keysym == 'k':
                const_3 = 1000
                const_2 = 1000  # pitch
                const_1 = 1000  # roll
                print("kill")
                print("\nThrottle value - " + str(const_3))
                print('Pitch value - ' + str(const_1))
                print('Roll value - ' + str(const_2))

            elif event.keysym == 'w' and const_3 < 2000:
                const_3 += amt
                print_fn_2()



            elif event.keysym == 's' and const_3 > 1040:
                const_3 -= amt
                print_fn_2()

        else:
            if event.keysym == 'Up':
                const_2 -= amt_2
                print("\nForward")
                print_fn_2()
                global m
                m = 1

            elif event.keysym == 'Down':
                const_2 += amt_2
                print("\nBackward")
                print_fn_2()
                global m
                m = 1

            elif event.keysym == 'Left':
                const_1 -= amt_2
                print("Left")
                print_fn_2()
                global m
                m = 1

            elif event.keysym == 'Right':
                const_1 += amt_2
                print("\nRight")
                print_fn_2()
                global m
                m = 1


#    else:  # -- non standard keys

# if event.keysym == 'Up':
#   vehicle.channels.overrides[2] -= amt  # pitch-control =  nose down (to go forward)
#  print("forward, on throttle ", (int(vehicle.channels.overrides[3])))

# elif event.keysym == 'Down':
#    vehicle.channels.overrides[2] += amt  # pitch-control =  nose up (to go backword)
#    print("backward, on throttle ", (int(vehicle.channels.overrides[3])))

# elif event.keysym == 'Left':
#    vehicle.channels.overrides[1] -= amt  # roll-control = left (move leftwards)
#    print("left, on throttle ", (int(vehicle.channels.overrides[3])))

# elif event.keysym == 'Right':
#    vehicle.channels.overrides[1] += amt  # roll control = right (move leftwards)
#    print("right, on throttle ", (int(vehicle.channels.overrides[3])))


def key_down(event):
    global const_1
    global const_2
    global const_3

    if m == 1:
        const_1 = 1500
        const_2 = 1500
        print_fn_2()
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
# root.bind_all('<Key>', key)
tk.Button(root, text="Quit", command=root.destroy).pack()
root.mainloop()
