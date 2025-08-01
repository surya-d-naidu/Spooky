# esp32_mpu6050.py
"""
MPU6050 reading for ESP32 (MicroPython)
Returns pitch and roll in degrees for stabilization.
"""

from machine import I2C, Pin
import math
import time

# MPU6050 I2C address
MPU6050_ADDR = 0x68

class MPU6050:
    def __init__(self, i2c):
        self.i2c = i2c
        # Wake up MPU6050
        self.i2c.writeto_mem(MPU6050_ADDR, 0x6B, b'\x00')

    def read_raw_accel_gyro(self):
        # Read 14 bytes: accel (6), temp (2), gyro (6)
        data = self.i2c.readfrom_mem(MPU6050_ADDR, 0x3B, 14)
        ax = int.from_bytes(data[0:2], 'big', signed=True)
        ay = int.from_bytes(data[2:4], 'big', signed=True)
        az = int.from_bytes(data[4:6], 'big', signed=True)
        gx = int.from_bytes(data[8:10], 'big', signed=True)
        gy = int.from_bytes(data[10:12], 'big', signed=True)
        gz = int.from_bytes(data[12:14], 'big', signed=True)
        return ax, ay, az, gx, gy, gz

    def get_pitch_roll(self):
        ax, ay, az, gx, gy, gz = self.read_raw_accel_gyro()
        # Convert to g
        ax /= 16384.0
        ay /= 16384.0
        az /= 16384.0
        # Calculate pitch and roll
        pitch = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
        roll = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
        return pitch, roll

# Example usage
if __name__ == '__main__':
    # ESP32 I2C pins (adjust as needed)
    i2c = I2C(0, scl=Pin(22), sda=Pin(21))
    mpu = MPU6050(i2c)
    while True:
        pitch, roll = mpu.get_pitch_roll()
        print('Pitch: {:.2f} Roll: {:.2f}'.format(pitch, roll))
        time.sleep(0.1)
