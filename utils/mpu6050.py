import smbus
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# MPU6050 registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# Initialize I2C bus
bus = smbus.SMBus(1)  # Use 0 for older Raspberry Pi models

# Wake up the MPU6050
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32767:
        value -= 65536
    return value

def read_mpu6050():
    accel_x = read_raw_data(ACCEL_XOUT_H)
    accel_y = read_raw_data(ACCEL_XOUT_H + 2)
    accel_z = read_raw_data(ACCEL_XOUT_H + 4)
    gyro_x = read_raw_data(ACCEL_XOUT_H + 8)
    gyro_y = read_raw_data(ACCEL_XOUT_H + 10)
    gyro_z = read_raw_data(ACCEL_XOUT_H + 12)
    
    print(f"Accel X: {accel_x} | Accel Y: {accel_y} | Accel Z: {accel_z}")
    print(f"Gyro X: {gyro_x} | Gyro Y: {gyro_y} | Gyro Z: {gyro_z}")
    print("------------------------------------")
    
    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z

# Define cuboid (breadboard) vertices
cuboid_vertices = np.array([
    [-1, -0.5, -0.2], [1, -0.5, -0.2], [1, 0.5, -0.2], [-1, 0.5, -0.2],
    [-1, -0.5, 0.2], [1, -0.5, 0.2], [1, 0.5, 0.2], [-1, 0.5, 0.2]
])

def plot_cuboid(ax, vertices):
    edges = [(0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), (0,4), (1,5), (2,6), (3,7)]
    for edge in edges:
        ax.plot3D(*zip(*vertices[list(edge)]), color='b')

# Setup 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.ion()
plt.show()

while True:
    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = read_mpu6050()
    
    # Normalize accelerometer data to estimate orientation
    accel_vector = np.array([accel_x, accel_y, accel_z])
    accel_vector = accel_vector / np.linalg.norm(accel_vector)
    
    # Convert to rotation angles
    roll = np.arctan2(accel_vector[1], accel_vector[2])
    pitch = np.arctan2(-accel_vector[0], np.sqrt(accel_vector[1]**2 + accel_vector[2]**2))
    
    # Create rotation matrix
    rotation_matrix = R.from_euler('xyz', [roll, pitch, 0]).as_matrix()
    rotated_vertices = np.dot(cuboid_vertices, rotation_matrix.T)
    
    ax.clear()
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plot_cuboid(ax, rotated_vertices)
    plt.draw()
    plt.pause(0.1)
