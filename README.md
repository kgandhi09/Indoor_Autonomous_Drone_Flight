Indoor Autonomous Drone Flight

>> This method utilizes the serial communication between raspberry pi and Pixhawk. 

>> To avoid the problem of GPS in an indoor environment this code uses channel
 overrides method and manipulates the PWM values of corresponding Channel.


1. Serial Communication between raspberry pi and Pixhawk
	1.1 Hardware


      1.2 Establishing serial communication between pi and pixhawk
            ▪ Setting Up Pixhawk
                • Necessary paramters to change after installing firmware and doing calibration





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
                          
        1.3  Check the connection between rpi and pixhawk:
    • Open the terminal
    • type : ‘sudo mavproxy.py --master=/dev/ttyS0 --baudrate 921600 --aircraft MyCopter’
    • it should show some messages including ‘Initializing APM’ and others as well.
    • If it shows link 1 down, there might be some issues with connections or other steps mentioned above.
			
 
                  


2. PPM Communication between raspberry pi and Pixhawk.

    • In this case raspberry pi acts as controller for the drone and raspberry pi itself generates ppm signals. 1 PPM signal consists of 8 different pwm signals for 8 different channels. Channel 1 being pitch, channel 2 being roll, channel 3 being throttle and channel 4 being yaw. Other four channels are not used in the drone currently so their signals doesn’t matter.
    • Each channel’s pwm value ranges from 1000 to 2000, which means if signal is at 1500 drone should be stable for respective channels. In case of throttle(channel 3) 1000 means 0% throttle and 2000 means 100% throttle.
      
    • Hardware connection:
        ◦ Any GPIO pin would go in signal pin of pixhawk’s rc receiver input (should be changed in code acc to connection).
        ◦ Ground pin from rpi to ground pin of pixhawk’s rc receiver input. 

FIRST ATTEMPT:

First strategy was clearly to add keyboard controls to the drone using rc channels overrides via mavlink protocol. This code did not include any gyro stabilization and supplying ppm signals to drone. Entire idea was to control roll, pitch, yaw and throttle using keyboard controls.

Channels overrides	
    • Channels overrides is an inbuilt function provided in dronekit which controls single channel’s pwm value.
    • Under Dronekit module in python class heirarchy is as follows:
        ◦ Dronekit>Vehicle>channels
        ◦ Inside this class override functions is used which takes in an element ranging from [1,2,3....8]. These elements are channel numbers.
        ◦ Object can be defined as vehicle.channels.overrides[3] = 1500 #Eamle
            ▪ Here channel 3 is set to 1500 which means 50% throttle.
      
              

    • Inside Autonomous_Drone_Flight > keyboard_controls_without_gyro&ppm 
        ◦ keyboard_controls_v1.py, keyboard_controls_v2.py, keyboard_controls_v3.py uses same method of using channels overrides as main source of controlling drone. 
      
      Conclusion: So to say, this method was not efficient at all as mavlink protocol took too much time when key was pressed and channel values were updated too late. Drone did not takeoff at all by this method.

SECOND ATTEMPT:

After keyboard controls, decided to use of gyro for self balancing instead of keyboard keys. Only throttle value was controlled by keyboard.

    • Hardware:
      

    • Use of external gyro:
        ◦ Connected external mpu6050 gyro as there was no access to pixhawk’s internal gyro.
        ◦ External gyro was not giving proper and stable values. Fluctuations sometimes ranged from -40 deg to + 40 deg even in stable condition.
        ◦ In Autonomous_Drone_Flight > gyro_stable.py external gyro is used with vehicle channel overrides. 
        ◦ Neither gyro values were good nor channel values were.  

Conclusion: Drone did not takeoff at all. Gyro values were very unstable and mavlink protocol form of communication took too long to update channel values. Decided to somehow access pixhawk’s internal gyro which had built-in Extended Kalman Filter.

THIRD ATTEMPT:

Third strategy was clearly not to use any external gyro and any other hardware. In this attempt, got the access of pixhawk’s internal gyro.

    • Pixhawk’s Internal Gyro accesss:
        ◦ internal gyro can be accessed though dronekit python module under attitude class.
        ◦ Class heirarchy is as follow:
            ▪ Dronekit > Vehicle > attitude 
        ◦ Object can be defined as ‘pitch = vehicle.attitude.pitch’ and ‘roll = vehicle.attitude.roll’
            ▪ vehicle being dronekit object
        ◦ inbuilt gyro values ranges from (-1 to +1). -1 being actual -60 degrees and +1 being actual +60 degrees.
          
    • This code flow used mavlink protocol to read gyro values from internal gyro and mavlink protocol for vehicle channel overrides.
        ◦ In Autonomous_Drone_Flight > simple_takeoff_v1.py internal gyro is used with vehicle channel overrides.

Conclusion: This method provided best results for gyro values using mavlink, but as before mavlink took too lnog to update channel values. Vehicle took off, was pretty much stable but not as desired. To take the drone to its best performance, decided to use mavlink protocol for sensing part,  but use ppm signals for controls part. This is where ppm connection betweeen rpi and pixhawk comes into action.

FOURTH ATTEMPT:

In this attempt, code used same logic to implement self-stabilization as we used before with external gyro, but for the controls part, instead of using mavlink for channel overrides we supplied ppm increment/decrement for self-stabilization.

    • Use of internal gyro is showed in the previous attempt
    • In Autonomous_Drone_Flight > simple_takeoff_v2.py internal gyro is used with ppm signals.

Conclusion: This gave the best performance so far and we were ready to move ahead with the navigation part.

FIFTH ATTEMPT:

Found version of pixhawk of firmware which supported Marvel Mind and can be used as Indoor GPS with pixhawk.

There were some issues which created interferences between telemetry and marvel mind as both were being operated on same frequency 440 mHZ. 

Conclusion : Drone did not read proper GPS values, lag in reading values. So decided to use cameras for navigation part.

SIXTH ATTEMPT:

This startegy consisted of using cameras and creating custom coordinate systems for getting current coordinates of drone using color detection and putting marker on the drone. This attempt also included use of previous method of self-stabilzation.

This method consists of two machines for communication and navigation of drone. Every math and computation is run on ground station whereas drone only sends gyro values and receives ppm signals after computation happens on ground station.

Drone:
    • Drone initially takes the reference of ground which is indicated by ‘storing gyro values for reference!’  in the code. 
    • After sending the init list of gyro value drone enters into three threads: send(), receive(), navigate()
        ◦ Send() - Send() continously sends the gyro values for self-balancing computation which happens at ground station
        ◦ Receive() - Receive() continously receives the ppm signals after balancing and computaiton happens at ground station.
        ◦ Navigate() - Navigate() keeps on updating the received ppm signals.


Ground Station:

Three main aspects of ground station are camera_detection(), change(), gyro()

    • Once it receives initial gyro reference list, it keeps on receiving gyro values and gyro() computes current gyro x and y axis’ values with the reference list and keeps on balancing it until it reaches stable condition.
    • camera_detection() runs a loop where it draws contours around the markers which in turn gives the center value of marker which is also drone’s center value. It then further computes to give the current coordinates of drone. 
    • Also a mouse callback function is called in the loop which gives new-coordinates where the mouse is clicked. Which also acts as the desired position for the drone.  
    • After the mouse is clicked and new_coordinateds are updated change() compares the drone’s current coordinates with the new mouse clicked coordinates. 
    • Change() then increments/decrements ppm_values and sends it to drone via socket tcp communication which acts as a navigation.

Autonomous_Drone_Flight > camera_controls > drone_data_v3.py is the final code.

Conclusion: It has given the best stabilization performance till now. For making throttle autonomous ultrosonic sensor will be required. For the navigation part more debugging and study of drone’s behaviour is rquired.

# --------------------------------------------- Author : Kushal Gandhi ------------------------------------------------------------------------------------------------------------#
