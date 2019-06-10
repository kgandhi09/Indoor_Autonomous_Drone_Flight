
import smbus # import SMBus module of I2C from time import sleep # import
from time import sleep
import math


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


def MPU_Init():
    # write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    # Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)

    # concatenate higher and lower value
    value = ((high << 8) | low)
 
    # to get signed value from mpu6050
    if (value > 32768):
        value = value - 65536
    return value

def dist(x,y):
   return math.sqrt((x*x) + (y*y))

def get_x_rotation(x,y,z):
   radians = math.atan2(x, dist(y,z))
   return -math.degrees(radians)

def get_y_rotation(x,y,z):
   radians = math.atan2(y, dist(x,z))
   return -math.degrees(radians)

bus = smbus.SMBus(1)  # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68  # MPU6050 device address

MPU_Init()


print (" Reading Data of Gyroscope and Accelerometer")

x = 1500
y = 1500

amt_2 = 1

Dx = 0
Dy = 0

list_x = []
list_y = []

count = 0

def assign_gyro_values():

    global Dx
    global Dy

    # Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)

    # Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)

    # Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0

    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0

    Dx = get_x_rotation(Ax, Ay, Az)
    Dy = get_y_rotation(Ax, Ay, Az)

    list_x.append(Dx)
    list_y.append(Dy)

    print('\n')
    print("X Rotation = %.2f" % Dx)
    print("Y Rotation = %.2f" % Dy)

def sensitivity_factor(num):
    if -1 < num < 0:
	return -(5*num)

    if 0 < num < 1:
	return (5*num)

    if num <= -1:
	return -((65*num)/100)

    if num >= 1:
    	return (65*num)/100

def gyro_balance(Gx, Gy):

    global x
    global y

    if min_x < Gx < max_x and min_y < Gy < max_y:
	x = 1500
	y = 1500

    else:

        if Gx > max_x + sensitivity_factor(max_x):
    	    x = 1470

        if Gx < min_x - sensitivity_factor(min_x):
	    x = 1530

        if Gy > max_y + sensitivity_factor(max_y):
	    y = 1470

        if Gy < min_y - sensitivity_factor(min_y):
	    y = 1530

    print("X Axis = " + str(x))
    print("Y Axis = " + str(y))
    #print(Gx > max_x + sf(max_x)) # ---- pos x axis
    #print(Gx < min_x - sf(min_x)) # ---- neg x axis
    #print(Gy > max_x + sf(max_y)) # ---- pos y axis
    #print(Gy < min_x - sf(min_y)) # ---- neg y axis


print("Callibrating external gyro sensor")

sleep(1)

while True:

    count += 1
    assign_gyro_values()

    max_x = round(max(list_x), 2)
    min_x = round(min(list_x), 2)
    max_y = round(max(list_y), 2)
    min_y = round(min(list_y), 2)

    if count > 1000:
	break

sleep(1)

print("Calibrate x and y min_max values - ")
print("\n")
print("Max X Value = " + str(max_x))
print("Min X Value = " + str(min_x))
print("Max Y Value = " + str(max_y))
print("Min Y Value = " + str(min_y))
print("\n")
print("Initializing gyro balance for drone")

sleep(4)

while True:

    assign_gyro_values()
    gyro_balance(Dx, Dy)

