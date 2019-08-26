import cv2
import numpy as np
import serial
import time
import math
import pickle
import socket
import readchar
from pynput.keyboard import Key, Controller, Listener
import threading
import json
import pickle

host = '192.168.0.101' # ip of raspberry pi
port = 9988 


# inittiates socket connection
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))

# function for formating bytes type data to list from incoming socket communication
# Not used currently, instead using json loads and dump for formatting
def format(byte):
    str = byte.decode('utf-8')
    str = str.translate({ord('['): None})
    str = str.translate({ord(']'): None})
    list = str.split(', ')
    final_list = []
    i = 0
    while True:
        final_list.append(float(list[i]))
        i += 1
        if i > len(list) - 1:
            break

    return final_list


print("\nGyro Values!")

reply = s.recv(1024) # receives initial gyro values for reference

gyro_list = json.loads(reply)

# Gyro values for reference received initially from drone through socket
min_x = gyro_list[0]  #-- maximum gyro value on x axis in degrees to be in stable condition
max_x = gyro_list[1]  #-- minimum gyro value on x axis in degrees to be in stable condition
min_y = gyro_list[2]  #-- maximum gyro value on y axis in degrees to be in stable condition
max_y = gyro_list[3]  #-- minimum gyro value on y axis in degrees to be in stable condition

print('\n'+"Max X Value = " + str(max_x))
print("Min X Value = " + str(min_x))

print('\n'+"Max Y Value = " + str(max_y))
print("Min Y Value = " + str(min_y))

ch_1 = 1500   # pitch (Y Axis)
ch_2 = 1500   # roll  (X Axis)
ch_3 = 1000   # Throttle (Z Axis)
ch_4 = 1500   # Yaw (Orientation)
ch_5 = 1500
ch_6 = 1500
ch_7 = 1500
ch_8 = 1500

ppm_signals = [ch_1, ch_2, ch_3, ch_4, ch_5, ch_6, ch_7, ch_8]  # init ppm signal list
stable_ppm_signals = [ch_1, ch_2, ch_3, ch_4, ch_5, ch_6, ch_7, ch_8]

print(ppm_signals)

gyro_flag = False
takeoff_flag = False

sf = 2 # +/- error for gyro stabilization

# Takeoff by incrementing and decrementing throttle value by 10 units by up and down key arrow press.
# param, key
# key = keyboard pressed
def takeoff(key):
    global stable_ppm_signals
    global gyro_flag
    global ppm_signals
    amt = 10

    # increases throttle value by amt(10)
    if key == Key.up:
        ppm_signals[2] += amt
        #print('Up')
        #if gyro_flag:
        reply = str(ppm_signals).encode('utf-8')
        s.send(reply)
    # decreases throttle value by amt(10)
    if key == Key.down:
        ppm_signals[2] -= amt
        #print('Down')
        #if gyro_flag:
        reply = str(ppm_signals).encode('utf-8')
        s.send(reply)
    # pressing shift will stop auto gyro stabilization and throttle up-down
    if key == Key.shift:
        gyro_flag = True
        stable_ppm_signals = ppm_signals.copy()
        #lis.stop()

    if key == Key.esc:
        stable_ppm_signals[2] = 1000

    if key == Key.end:
        vehicle_mode = 'LAND'
        s.send(vehicle_mode.encode('utf-8'))

# defines the position condition in which drone is stable
# param, Gx, Gy
# Gx - current(real time) X rotation gyro value
# Gy - current(real time) Y rotation gyro value
def stable_pos(Gx, Gy):
    result = False
    global sf
    if min_x - sf < Gx < max_x + sf and min_y - sf < Gy < max_y + sf:
        result = True
    return result

# Auto stabilization function
# Counter values if drone is not in stable position
# param, Gx, Gy
# Gx - current(real time) X rotation gyro value
# Gy - current(real time) Y rotation gyro value
def stabilize(Gx, Gy):
    i = 1
    global channel_list

# Below listed are the unstable conditions in four directions
    #Left motion
    if Gx > max_x + sf: 
        ppm_signals[0] -= i

    #Right motion
    if Gx < min_x - sf: 
        ppm_signals[0] += i
    
    #Backward Motion
    if Gy > max_y + sf: 
        ppm_signals[1] -= i
    
    #Forward Motion
    if Gy < min_y - sf: 
        ppm_signals[1] += i
	    
	
# continuously receives gyro values and sends ppm changes to the drone
# If throttle in stable condition - continue throttle value
# If throttle not in stable condition - stabilize and again continue throttle if stabilize true
def gyro():

    global ppm_signals
    global gyro_flag
    global reply 
    while True:
        msg=s.recv(1024)
        
        try:
            curr_gyro_list = json.loads(msg)
            new_pitch = curr_gyro_list[0]
            new_roll = curr_gyro_list[1]
    
        except:
            pass
            
        if not gyro_flag:
            if stable_pos(new_roll, new_pitch):
                #print('stable')
                reply = str(ppm_signals).encode('utf-8') 
                s.send(reply)
            elif not stable_pos(new_roll, new_pitch):
                #print('unstable')
                stabilize(new_roll, new_pitch)
                reply = str(ppm_signals).encode('utf-8')
                s.send(reply)
        
        
        time.sleep(0.07)

#---------------------------------------------------------------------------------------------- #
def cnt_idfy_send(colour,cnt):

    try:
        
        if (colour =='orange'):

            M = cv2.moments(cnt)
            cX = int(M["m10"]/M["m00"])
            cY = int(M["m01"]/M["m00"])
            cv2.putText(frame,"L",(cX,cY),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)

        if (colour =='sb'):

            M = cv2.moments(cnt)
            cX = int(M["m10"]/M["m00"])
            cY = int(M["m01"]/M["m00"])
            cv2.putText(frame,"R",(cX,cY),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)

         

        else:
            pass
           

    except:
        pass

    else:
        pass


last_time=0

oX = 0
oY = 0

bX = 0
bY = 0

center_x = 0
center_y = 0

curr_coord_x = 0
curr_coord_y = 0

circles = []

count = 0

reached = False

# mouse click event
def mouse_drawing(event, x, y, flags, params):
    global count
    global circles
    global reply_flag
    if event == cv2.EVENT_LBUTTONDOWN:
        #print("Left click")
        circles.append((x, y))
        count += 1

# changes the drone ppm signals according to the latest left mouse click
# assign new coordinates to the drone (new x and new y where the mouse is clicked)
# If new x and new y different from current x and y, then changes ppm signals and sends to drone via socket
# Else sends stable ppm signals to the drone via socket
def change():
    global stable_ppm_signals
    global state
    global curr_coord_x,curr_coord_y
    global new_coord_x, new_coord_y
    global ppm_signals

    
    if new_coord_x > curr_coord_x and (not state):
        #print('1')
        #ppm_signals[0] += 1 #Roll
        ppm_signals[0] = stable_ppm_signals[0] + 30
        print('on the way!')
        s.send(str(ppm_signals).encode('utf-8'))
    if new_coord_x  < curr_coord_x and (not state):
        #print('2')
        #ppm_signals[0] -= 1 #Roll
        ppm_signals[0] = stable_ppm_signals[0] - 30
        print('on the way!')
        s.send(str(ppm_signals).encode('utf-8'))
    if new_coord_y > curr_coord_y and (not state):
        #print('3')   
        #ppm_signals[1] -= 1 #Pitch
        ppm_signals[1] = stable_ppm_signals[1] - 30
        print('on the way!')
        s.send(str(ppm_signals).encode('utf-8'))
    if new_coord_y < curr_coord_y and (not state):
        #print('4')
        #ppm_signals[1] += 1 #Pitch
        ppm_signals[1] = stable_ppm_signals[1] + 30
        print('on the way!')
        s.send(str(ppm_signals).encode('utf-8'))
    
    
    if state: 
        print('reached')
        s.send(str(stable_ppm_signals).encode('utf-8'))

# Detects and draws around the marker attached to the drone
# Calculates current coordinates and compares with new coordinates(mouse click)
# Changes ppm signals according to new mouse click coordinates by calling change()
def cam_detection():
    global ppm_signals
    global state
    global count
    global oX,oY
    global bX,bY
    global cap
    global center_x, center_y
    global curr_coord_x, curr_coord_y
    global new_coord_x, new_coord_y
    global last_time
    global circles
    global frame
    global list_ppm
    global setup
    
    print('cam-detection started!')

    cap = cv2.VideoCapture(0)
    font = cv2.FONT_HERSHEY_COMPLEX
    
    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", mouse_drawing)
    
    

    while True:

        ret,frame = cap.read()
        cv2.setMouseCallback("Frame", mouse_drawing)
        
    
        if gyro_flag:
            cv2.circle(frame, (320, 240), 30, (0,0,255), 2)
            cv2.line(frame, (320, 0), (320, 480) , (0,255,0), 1)
            cv2.line(frame, (0, 240), (640, 240), (0,255,0), 1)
            cv2.line(frame, (320,120), (330, 130), (0,0,0), 2)
            cv2.line(frame, (320,120), (310, 130), (0,0,0), 2)
            cv2.putText(frame, "Front this side!", (255,120), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)

            current_time= int(round(time.time()*1000))
            
            if(current_time-last_time>50):
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                #### Step 1############

                #defining the Range of orange color
                orange_lower=np.array([29, 64, 255],np.uint8)
                orange_upper=np.array([68, 149, 255],np.uint8)

                #defining the Range of Sky Blue color
                sb_lower=np.array([91, 97, 190],np.uint8)
                sb_upper=np.array([147, 255, 255],np.uint8)

                ####### Step 2#############

                orange=cv2.inRange(hsv,orange_lower,orange_upper)

                sb=cv2.inRange(hsv,sb_lower,sb_upper)

                ######### Step 3 ########

                kernal = np.ones((5 ,5), "uint8")


                orange=cv2.dilate(orange,kernal)
                res3=cv2.bitwise_and(frame, frame, mask = orange)
                colour0 = "orange"

                sb=cv2.dilate(sb,kernal)
                res3=cv2.bitwise_and(frame, frame, mask = sb)
                colour3 = "sb"

            
                ######## Step 4 ##############

                

                # Contours detection for orange
                contours = cv2.findContours(orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    approx = cv2.approxPolyDP(cnt, 0.0000001*cv2.arcLength(cnt, True), True)
                    if area > 50:
                        cv2.drawContours(frame, [approx], 0, (0, 0, 0), 2)
                        M = cv2.moments(cnt)
                        xb= int(M["m10"]/M["m00"])
                        yb= int(M["m01"]/M["m00"])
                        oX = xb
                        oY = yb
                        cv2.line(frame, (xb,yb), (320, 240), (0,0,255), 2)
                        cv2.circle(frame,(xb,yb), 1, (0,0,0), -1)
                        cnt_idfy_send(colour0,cnt)

                # Contours detection for sb
                contours1 = cv2.findContours(sb, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
                for cnt in contours1:
                    area = cv2.contourArea(cnt)
                    approx = cv2.approxPolyDP(cnt, 0.0000001*cv2.arcLength(cnt, True), True)
                    if area > 100:
                        cv2.drawContours(frame, [approx], 0, (0, 0, 0), 2)
                        M = cv2.moments(cnt)
                        xb= int(M["m10"]/M["m00"])
                        yb= int(M["m01"]/M["m00"])
                        bX = xb
                        bY = yb
                        cv2.line(frame, (xb,yb), (320, 240), (255,0,0), 2)
                        cv2.circle(frame,(xb,yb), 1, (0,0,0), -1)
                        cnt_idfy_send(colour3,cnt)

                center_x = int((oX + bX)/2)
                center_y = int((oY + bY)/2)

                #print(center_x, center_y)
                
                curr_coord_x = center_x
                curr_coord_y = center_y
                
                cv2.putText(frame,"(0,0)",(320,240),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
                cv2.putText(frame, '(' + str(curr_coord_x) + ' , ' + str(curr_coord_y) + ')',(center_x, center_y),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
                cv2.circle(frame, (curr_coord_x, curr_coord_y), 25, (0,255,0), 2)

                new_coord_x = curr_coord_x
                new_coord_y = curr_coord_y

                cv2.putText(frame, str(count), (new_coord_x, new_coord_y), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)

                for x,y in circles:
                    cv2.circle(frame, (x,y) , 5, (0, 0, 255), -1)


                    new_coord_x = x
                    new_coord_y = y

                # cv2.line(frame, (coord_x,coord_y), (new_coord_x,new_coord_y), (0,0,255), 2)            
                
                
                ######## End ############


                last_time=current_time

            #cv2.putText(frame, str(count), (new_coord_x, new_coord_y), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)    
            
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
            elif key == ord("d"):
                circles = []
            
            state = new_coord_x - 25 < curr_coord_x < new_coord_x + 25 and new_coord_y - 25 < curr_coord_y < new_coord_y + 25

            change()
            #s.send(str(ppm_signals).encode('utf-8'))
#---------------------------------------------------------------------------------------------- #

if __name__ == '__main__':
    
    t1 = threading.Thread(target = gyro)
    t2 = threading.Thread(target = cam_detection)

    t1.start()
    t2.start()

    lis = Listener(on_press = takeoff)
    lis.start()
    lis.join()
