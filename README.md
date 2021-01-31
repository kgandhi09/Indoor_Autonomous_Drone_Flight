# Indoor Autonomous Drone Flight #

Since, the drone has to be used in indoor environments and for applications such as warehouse management, it doesn't have access to GPS signals and thus, waypoint navigation system using MissionPlanner and self-stabilzation mode cannot be used. This method utilizes serial communication between RaspberryPi and Pixhawk 2 to generate and manipulate PWM signals for stabilization and navigation. Program has self-stabilization feature that uses Pixhwak's inetrnal gyro data. Has a two way tcp socket communication between drone and ground station. Ground station uses an overhead camera and a computer vision algorithm to detect and localize the drones through colored markers attached on the top of the drone.

## Serial Communication between raspberry pi and Pixhawk
### Establishing serial communication between pi and pixhawk
#### Setting Up Pixhawk

                • Installing necesarry packages in rpi
                    ◦ sudo apt-get update
                    ◦ sudo apt-get install screen python-wxgtk2.8 python-matplotlib python-opencv python-pip python-numpy python-dev libxml2-dev libxslt-dev python-lxml
                    ◦ sudo pip install future
                    ◦ sudo pip install pymavlink
                    ◦ sudo pip install dronekit
                    ◦ sudo pip install mavproxy
                  
                • Disable the OS control of the serial port
                    ◦ type “sudo raspi-config” in rpi console
                    ◦ navigate to serial and disable the os use of serial connection
                    ◦ On Rpi, the uart serial connection must be disabled by default. In order to enable uart serial connection. Type the following on terminal:
                        ▪ sudo nano /boot/config.txt
                        ▪ set ‘enable_uart = 1’
                        ▪ reboot
                        ▪ open terminal and type : ‘cd /dev/’ and then ‘ls’, port ttyS0 should appear
                          
### Check the connection between rpi and pixhawk:
    • Open the terminal
    • type : ‘sudo mavproxy.py --master=/dev/ttyS0 --baudrate 921600 --aircraft MyCopter’
    • it should show some messages including ‘Initializing APM’ and others as well.
    • If it shows link 1 down, there might be some issues with connections or other steps mentioned above.
                  

## PPM Communication between raspberry pi and Pixhawk.

    • In this case raspberry pi acts as controller for the drone and raspberry pi itself generates ppm signals. 1 PPM signal consists of 8 different pwm signals for 8 different channels. Channel 1 being pitch, channel 2 being roll, channel 3 being throttle and channel 4 being yaw. Other four channels are not used in the drone currently so their signals doesn’t matter.
    • Each channel’s pwm value ranges from 1000 to 2000, which means if signal is at 1500 drone should be stable for respective channels. In case of throttle(channel 3) 1000 means 0% throttle and 2000 means 100% throttle.
    • Hardware connection:
        ◦ Any GPIO pin would go in signal pin of pixhawk’s rc receiver input (should be changed in code acc to connection).
        ◦ Ground pin from rpi to ground pin of pixhawk’s rc receiver input. 


This startegy consists of using cameras and creating custom coordinate systems for getting current coordinates of drone using color detection and putting marker on the drone. This attempt also includes use of self-stabilzation using pixhawk's internal gyro.

This method consists of two machines for communication and navigation of drone. Every math and computation is run on ground station whereas drone only sends gyro values and receives ppm signals before and after computation happens respectively on ground station.

## Drone:
    • Drone initially takes the reference of ground which is indicated by ‘storing gyro values for reference!’  in the code. 
    • After sending the init list of gyro value drone enters into three threads: send(), receive(), navigate()
        ◦ Send() - Send() continously sends the gyro values for self-balancing computation which happens at ground station
        ◦ Receive() - Receive() continously receives the ppm signals after balancing and computaiton happens at ground station.
        ◦ Navigate() - Navigate() keeps on updating the received ppm signals.


## Ground Station:

Three main aspects of ground station are camera_detection(), change(), gyro()
### Flow
    • Once it receives initial gyro reference list, it keeps on receiving gyro values and gyro() computes current gyro x and y axis’ values with the reference list and keeps on balancing it until it reaches stable condition.
    • camera_detection() runs a loop where it draws contours around the markers which in turn gives the center value of marker which is also drone’s center value. It then further computes to give the current coordinates of drone. 
    • Also a mouse callback function is called in the loop which gives new-coordinates where the mouse is clicked. Which also acts as the desired position for the drone.  
    • After the mouse is clicked and new_coordinateds are updated change() compares the drone’s current coordinates with the new mouse clicked coordinates. 
    • Change() then increments/decrements ppm_values and sends it to drone via socket tcp communication which acts as a navigation.

Conclusion: It has given the best stabilization performance till now. For making throttle autonomous ultrosonic sensor will be required. For the navigation part more debugging and study of drone’s behaviour is rquired.

## Link to the Demo:
https://www.youtube.com/watch?v=shfXqi58m1o

