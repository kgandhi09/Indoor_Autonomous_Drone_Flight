# --- writing of the header data for the text output file
def writeheader(dirpath):
    gyrodata = open(dirpath + "/gyro_out.txt", "w")
    gyrodata.write(str("Time"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("x-Angle"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("y-Angle"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("x-Rotation"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("y-Rotation"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("z-Rotation"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("x-Rotation-scaled"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("y-Rotation-scaled"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("z-Rotation-scaled"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("x-Acceleration"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("y-Acceleration"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("z-Acceleration"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("x-Acceleration-scaled"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("y-Acceleration-scaled"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("z-Acceleration-scaled"))
    gyrodata.write(str('\n'))

    gyrodata.write(str("[s]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[deg]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[deg]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[-]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[-]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[-]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[-]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[-]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[-]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[m/s2]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[m/s2]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[m/s2]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[-]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[-]"))
    gyrodata.write(str('\t'))
    gyrodata.write(str("[-]"))
    gyrodata.write(str('\n'))
    gyrodata.close()

def writedata(dirpath, counter):
    import smbus
    import math
    import time
    start_time = time.time()
    # Power management registers
    power_mgmt_1 = 0x6b
    # power_mgmt_2 = 0x6c

    gyrodata = open(dirpath + "/gyro_out.txt", "a")

    def read_byte(adr):
        return bus.read_byte_data(address, adr)

    def read_word(adr):
        high = bus.read_byte_data(address, adr)
        low = bus.read_byte_data(address, adr + 1)
        val = (high << 8) + low
        return val

    def read_word_2c(adr):
        val = read_word(adr)
        if (val >= 0x8000):
            #           return -((65535 - val) + 1)
            return -((65000 - val) + 1)
        else:
            return val

    def dist(a, b):
        return math.sqrt((a * a) + (b * b))

    def get_y_rotation(x, y, z):
        radians = math.atan2(x, dist(y, z))
        return -math.degrees(radians)

    def get_x_rotation(x, y, z):
        radians = math.atan2(y, dist(x, z))
        return math.degrees(radians)

    bus = smbus.SMBus(1)  # or bus = smbus.SMBus(1) for Revision 2 boards
    address = 0x68  # This is the address value read via the i2cdetect command

    # Now wake the 6050 up as it starts in sleep mode
    bus.write_byte_data(address, power_mgmt_1, 0)

    # print "gyro data"
    # print "---------"

    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)

    # print "gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / 131)
    # print "gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / 131)
    # print "gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / 131)

    # print
    # print "accelerometer data"
    # print "------------------"

    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)

    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0

    # print "accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled
    # print "accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled
    # print "accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled

    # print "x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    # print "y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

    x_rot = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    y_rot = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    end_time = time.time()
    cou_time = counter + end_time - start_time
    gyrodata.write(str("%s" % cou_time))
    gyrodata.write(str('\t'))
    gyrodata.write(str(x_rot))
    gyrodata.write(str('\t'))
    gyrodata.write(str(y_rot))
    gyrodata.write(str('\t'))

    gyrodata.write(str(gyro_xout))
    gyrodata.write(str('\t'))
    gyrodata.write(str(gyro_yout))
    gyrodata.write(str('\t'))
    gyrodata.write(str(gyro_zout))
    gyrodata.write(str('\t'))

    gyrodata.write(str(gyro_xout / 131))
    gyrodata.write(str('\t'))
    gyrodata.write(str(gyro_yout / 131))
    gyrodata.write(str('\t'))
    gyrodata.write(str(gyro_zout / 131))
    gyrodata.write(str('\t'))

    gyrodata.write(str(accel_xout))
    gyrodata.write(str('\t'))
    gyrodata.write(str(accel_yout))
    gyrodata.write(str('\t'))
    gyrodata.write(str(accel_zout))
    gyrodata.write(str('\t'))

    gyrodata.write(str(accel_xout_scaled))
    gyrodata.write(str('\t'))
    gyrodata.write(str(accel_yout_scaled))
    gyrodata.write(str('\t'))
    gyrodata.write(str(accel_zout_scaled))
    gyrodata.write(str('\t'))

    gyrodata.write(str('\n'))


gyrodata.close()