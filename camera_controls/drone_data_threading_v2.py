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

def mouse_drawing(event, x, y, flags, params):
    global count
    global circles
    global reply_flag
    if event == cv2.EVENT_LBUTTONDOWN:
        #print("Left click")
        circles.append((x, y))
        count += 1
        reply = b'mosue click!'
        reply_flag = 0
        
list_ppm = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]


state = False
setup=0

def change(list):
    global state
    global curr_coord_x,curr_coord_y
    global new_coord_x, new_coord_y
    global list_ppm

    
    if new_coord_x > curr_coord_x and (not state):
        #print('1')
        list[0] = 1530 #Roll
        print('on the way!')
    if new_coord_x  < curr_coord_x and (not state):
        #print('2')
        list[0] = 1470 #Roll
        print('on the way!')
    if new_coord_y > curr_coord_y and (not state):
        #print('3')   
        list[1] = 1470 #Pitch
        print('on the way!')
    if new_coord_y < curr_coord_y and (not state):
        #print('4')
        list[1] = 1530 #Pitch
        print('on the way!')
    
    
    elif state: 
        print('reached')
        list[0] = 1500
        list[1] = 1500

    return list



def cam_detection():

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
    #cv2.setMouseCallback("Frame", mouse_drawing)
    
    

    while True:
        if(setup):

            ret,frame = cap.read()
            cv2.setMouseCallback("Frame", mouse_drawing)

        

            cv2.circle(frame, (320, 240), 30, (0,0,255), 2)
            cv2.line(frame, (320, 0), (320, 480) , (0,255,0), 1)
            cv2.line(frame, (0, 240), (640, 240), (0,255,0), 1)
            cv2.line(frame, (320,120), (330, 130), (0,0,0), 2)
            cv2.line(frame, (320,120), (310, 130), (0,0,0), 2)
            cv2.putText(frame, "Front this side!", (255,120), cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)

            current_time= int(round(time.time()*1000))
            
            if(current_time-last_time>50):
                
                #print(cap.read())
                
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                #### Step 1############

                #defining the Range of orange color
                orange_lower=np.array([ 0, 89, 255],np.uint8)
                orange_upper=np.array([86, 192, 255],np.uint8)

                #defining the Range of Sky Blue color
                sb_lower=np.array([88, 58, 198],np.uint8)
                sb_upper=np.array([137, 140, 255],np.uint8)

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
            print(list_ppm)
            change(list_ppm)

host = '192.168.0.100'
port = 9988

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
conn,address= 0, 0


def setupServer():
    global s
    #global conn
    print("Socket created.")
    try:
        s.bind((host, port))
        print("bind")
    except socket.error as msg:
        print(msg)
    print("Socket bind complete.")
    #return s

def setupConnection(conn,address):
    #global conn,address
    setupServer()
    s.listen(1) # Allows one connection at a time.
    conn, address = s.accept()
    print("Connected to: " + address[0] + ":" + str(address[1]))
    return conn



def socket_comm():
    global conn
    global address
    global ppm_signals
    global setup
    global reply_list
    global reply_flag

    conn=setupConnection(conn,address)
    
   
    key = readchar.readkey()
    setup=0
    reply_flag=0

    data = conn.recv(1024)
    ppm_signals = data.decode("utf-8")
    print('hi')
    #print(ppm_signals)
    list_ppm = ppm_signals.split(", ")
    print(list_ppm)
    print(type(list_ppm))
    reply_list = b'GOT THE DATA!'
    conn.sendall(reply_list)

    '''
    while True:
       
        if True: #(setup==0):
            data = conn.recv(1024)
            ppm_signals = data.decode("utf-8")
            print('hi')
            #print(ppm_signals)
            #print(type(list_ppm))
            list_ppm = ppm_signals.split(", ")
            print(list_ppm)
            print(type(list_ppm))
            setup=1

        #else:
            reply_list = b'GOT THE DATA!' #list_ppm
            #reply = str(reply_list).encode("utf-8")
            conn.sendall(reply_list)
            reply_flag=1
            time.sleep(0.01)
        '''
    
    conn.close()
    
if __name__ == '__main__':
    
    t1 = threading.Thread(target = cam_detection)
    t2 = threading.Thread(target = socket_comm)
    t1.start()
    t2.start()

    
    
#cap.release()
cv2.destroyAllWindows()
