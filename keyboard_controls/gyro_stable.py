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


list_x = []
list_y = []

count = 0

def assign_gyro_values():

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
    print(x)
    print(y)

def gyro_balance(Gx, Gy):

    if Gx > max_x or Gx < min_x:
	if Gx > max_x:
	    x = 1470

	elif Gx < min_x:
	    x = 1530

    if Gy > max_y or Gy < min_y:
	if Gy > max_y:
	    y = 1470

	elif Gy < min_y:
	    y = 1530

    else:
	x = 1500
	y = 1500

print("Callibrating external gyro sensor")

sleep(1)

while True:

    count += 1
    assign_gyro_values()

    max_x = max(list_x)
    min_x = min(list_x)
    max_y = max(list_y)
    min_y = min(list_y)

    if count > 1000:
	break

print("Calibrate x and y min_max values - ")
print("\n")
print("Max X Value = " + str(max_x))
print("Min X Value = " + str(min_x))
print("Max Y Value = " + str(max_y))
print("Min Y Value = " + str(min_y))

while True:
    assign_gyro_values()
    
