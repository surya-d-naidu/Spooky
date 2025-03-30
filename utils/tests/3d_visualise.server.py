from flask import Flask, render_template
from flask_socketio import SocketIO
import time, math, threading
from mpu6050 import mpu6050  # pip install mpu6050-raspberrypi

app = Flask(__name__, template_folder="templates")
socketio = SocketIO(app, cors_allowed_origins="*")

# Initialize MPU6050 (assumes default I2C address 0x68)
sensor = mpu6050(0x68)

# Calibration values (set these via calibration routines or manually)
CALIBRATION = {
    "accel": {"x": 0, "y": 0, "z": 0},
    "gyro": {"x": 0, "y": 0, "z": 0}
}

# Global state for yaw and height
yaw = 0.0
height = 1.0

def compute_orientation(accel_data):
    """
    Compute roll and pitch (in degrees) from accelerometer data.
    These equations assume near-static conditions.
    """
    ax = accel_data["x"]
    ay = accel_data["y"]
    az = accel_data["z"]
    roll = math.atan2(ay, az) * 180 / math.pi
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    return roll, pitch

def sensor_loop():
    """Continuously read sensor data and emit updates via SocketIO."""
    global yaw, height
    last_time = time.time()
    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Get sensor data
        accel_data = sensor.get_accel_data()  # keys: x, y, z (in g)
        gyro_data = sensor.get_gyro_data()      # keys: x, y, z (assumed in deg/s)

        # Apply calibration offsets (if any)
        ax = accel_data["x"] - CALIBRATION["accel"]["x"]
        ay = accel_data["y"] - CALIBRATION["accel"]["y"]
        az = accel_data["z"] - CALIBRATION["accel"]["z"]
        corrected_accel = {"x": ax, "y": ay, "z": az}

        # Compute roll and pitch from accelerometer
        roll, pitch = compute_orientation(corrected_accel)

        # Integrate gyro Z for yaw (gyro_data["z"] in deg/s)
        yaw_rate = gyro_data["z"] - CALIBRATION["gyro"]["z"]
        yaw += yaw_rate * dt

        # For height, simulate a dynamic value based on vertical acceleration.
        # (Note: double-integrating acceleration is error-prone. In real applications,
        # use a dedicated sensor like an ultrasonic or lidar sensor.)
        vertical_accel = az * 9.81  # convert g to m/s²
        height += vertical_accel * dt * 0.001  # small factor to reduce drift
        height = max(0.5, min(height, 1.5))    # clamp height between 0.5 and 1.5 m

        # Compute the magnitude of the acceleration vector (as a proxy for force)
        force_magnitude = math.sqrt(ax**2 + ay**2 + az**2)

        # Package data to emit
        data = {
            "height": height,
            "acceleration": corrected_accel,
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
        socketio.sleep(0.05)  # update ~20 times per second

@app.route('/')
def index():
    return render_template('three_socket.html')

if __name__ == '__main__':
    sensor_thread = threading.Thread(target=sensor_loop)
    sensor_thread.daemon = True
    sensor_thread.start()
    socketio.run(app, host='0.0.0.0', port=5002, debug=True)

