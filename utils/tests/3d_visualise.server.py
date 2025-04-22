from flask import Flask, render_template
from flask_socketio import SocketIO
import time, math, threading
from mpu6050 import mpu6050  # pip install mpu6050-raspberrypi
import numpy as np

app = Flask(__name__, template_folder="templates")
socketio = SocketIO(app, cors_allowed_origins="*")

# Initialize MPU6050 (assumes default I2C address 0x68)
sensor = mpu6050(0x68)

# Calibration values (adjust as needed)
CALIBRATION = {
    "accel": {"x": 0, "y": 0, "z": 0},
    "gyro": {"x": 0, "y": 0, "z": 0}
}

height = 1.0

class KalmanFilter6D:
    def __init__(self):
        self.dt = 0.05
        self.x = np.zeros((6, 1))  # roll, pitch, yaw, bias_x, bias_y, bias_z
        self.P = np.eye(6) * 0.01
        self.F = np.eye(6)
        self.Q = np.eye(6) * 0.001
        self.H = np.zeros((2, 6))
        self.H[0, 0] = 1  # roll
        self.H[1, 1] = 1  # pitch
        self.R = np.eye(2) * 0.03

    def predict(self, gyro):
        omega = gyro.reshape((3, 1)) - self.x[3:6]
        self.x[0:3] += omega * self.dt
        self.F[0, 3] = -self.dt
        self.F[1, 4] = -self.dt
        self.F[2, 5] = -self.dt
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, accel):
        ax, ay, az = accel
        measured_roll = math.atan2(ay, az) * 180 / math.pi
        measured_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
        z = np.array([[measured_roll], [measured_pitch]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def get_orientation(self):
        return self.x[0, 0], self.x[1, 0], self.x[2, 0]  # roll, pitch, yaw

kf6d = KalmanFilter6D()

def sensor_loop():
    global height
    last_time = time.time()
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        kf6d.dt = dt

        # Sensor readings
        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()

        # Apply calibration
        ax = accel_data["x"] - CALIBRATION["accel"]["x"]
        ay = accel_data["y"] - CALIBRATION["accel"]["y"]
        az = accel_data["z"] - CALIBRATION["accel"]["z"]
        gx = gyro_data["x"] - CALIBRATION["gyro"]["x"]
        gy = gyro_data["y"] - CALIBRATION["gyro"]["y"]
        gz = gyro_data["z"] - CALIBRATION["gyro"]["z"]

        kf6d.predict(np.array([gx, gy, gz]))
        kf6d.update(np.array([ax, ay, az]))
        roll, pitch, yaw = kf6d.get_orientation()

        # Height estimation (fake for demo purposes)
        vertical_accel = az * 9.81
        height += vertical_accel * dt * 0.001
        height = max(0.5, min(height, 1.5))

        # Acceleration force
        force_magnitude = math.sqrt(ax**2 + ay**2 + az**2)

        data = {
            "height": height,
            "acceleration": {"x": ax, "y": ay, "z": az},
            "orientation": {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw
            },
            "force": {
                "magnitude": force_magnitude
            }
        }

        socketio.emit('sensor_update', data)
        socketio.sleep(0.05)

@app.route('/')
def index():
    return render_template('three_socket.html')

if __name__ == '__main__':
    sensor_thread = threading.Thread(target=sensor_loop)
    sensor_thread.daemon = True
    sensor_thread.start()
    socketio.run(app, host='0.0.0.0', port=5002, debug=True)
