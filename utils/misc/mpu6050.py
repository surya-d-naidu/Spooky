import smbus
import time

# Define I2C bus (1 for Raspberry Pi 2, 3, 4, etc.)
bus = smbus.SMBus(1)

# MPU-6050 I2C address
MPU_ADDR = 0x68

# MPU-6050 registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Wake up the MPU-6050
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        val -= 0x10000
    return val

def read_accel():
    accel_x = read_word(ACCEL_XOUT_H)
    accel_y = read_word(ACCEL_XOUT_H + 2)
    accel_z = read_word(ACCEL_XOUT_H + 4)
    return accel_x, accel_y, accel_z

def read_gyro():
    gyro_x = read_word(GYRO_XOUT_H)
    gyro_y = read_word(GYRO_XOUT_H + 2)
    gyro_z = read_word(GYRO_XOUT_H + 4)
    return gyro_x, gyro_y, gyro_z

while True:
    accel_x, accel_y, accel_z = read_accel()
    gyro_x, gyro_y, gyro_z = read_gyro()
    
    print(f"Accelerometer Data: X={accel_x}, Y={accel_y}, Z={accel_z}")
    print(f"Gyroscope Data: X={gyro_x}, Y={gyro_y}, Z={gyro_z}")
    
    time.sleep(1)
