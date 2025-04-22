
import smbus
import time
import math
from Kalman import KalmanAngle

# MPU6050 Constants
ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Math Constants
radToDeg = 57.2957786

# Kalman filters
kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

bus = smbus.SMBus(1)

# Kalman timing
last_time = time.time()

def MPU_Init():
    bus.write_byte_data(ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(ADDR, PWR_MGMT_1, 1)
    bus.write_byte_data(ADDR, CONFIG, 6)
    bus.write_byte_data(ADDR, GYRO_CONFIG, 24)
    bus.write_byte_data(ADDR, INT_ENABLE, 1)

MPU_Init()

def read_raw_data(reg):
    high = bus.read_byte_data(ADDR, reg)
    low = bus.read_byte_data(ADDR, reg+1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def get_data():
    ax = read_raw_data(ACCEL_XOUT_H)
    ay = read_raw_data(ACCEL_XOUT_H+2)
    az = read_raw_data(ACCEL_XOUT_H+4)

    roll  = math.atan2(ay, az) * radToDeg
    pitch = math.atan(-ax / math.sqrt(ay * ay + az * az)) * radToDeg

    return roll, pitch

def apply_kalman_filter(angles):
    global last_time
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    gx = read_raw_data(GYRO_XOUT_H) / 131.0
    gy = read_raw_data(GYRO_XOUT_H + 2) / 131.0

    roll, pitch = angles
    kx = kalmanX.getAngle(roll, gx, dt)
    ky = kalmanY.getAngle(pitch, gy, dt)

    return kx, ky

def get_deltaL(side, angles):
    roll, pitch = math.radians(angles[0]), math.radians(angles[1])
    h = side / 2
    corners = [(-h, -h), (h, -h), (h, h), (-h, h)]
    deltas = []

    for x, y in corners:
        z = x * math.sin(pitch) + y * math.sin(roll)
        deltas.append(z)

    return deltas

def set_angle_after_balance(angles, L1, L2):
    deltas = get_deltaL(L1 + L2, angles)
    result_angles = []

    for s in deltas:
        try:
            phi = math.acos((s**2 - L1**2 - L2**2) / (2 * L1 * L2))
            theta = math.atan2(s, L1 + L2 * math.cos(phi))
            result_angles.append((math.degrees(theta), math.degrees(phi)))
        except:
            result_angles.append((0.0, 0.0))  # fallback if unreachable
    return result_angles
