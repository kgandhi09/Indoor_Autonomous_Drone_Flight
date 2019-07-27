import cv2
import numpy as np
import serial 
import time
import math
import pickle
import socket
import readchar
from pynput.mouse import Button, Controller
import threading

mouse = Controller()
X = 0
Y = 0
cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_COMPLEX

def mouse_drawing(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        #print("Left click")
        circles.append((x, y))

cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", mouse_drawing)

circles = []

while True:

    ret, frame = cap.read()

    for x,y in circles:
        cv2.circle(frame, (x,y) , 5, (0, 0, 255), -1)
        X = x
        Y = y

    cv2.imshow("Frame", frame)

    key = cv2.waitKey(1)
    if key == 27:
        
        break

    elif key == ord("d"):
        circles = []

    print(X,Y)

