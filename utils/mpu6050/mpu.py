from Kalman import KalmanAngle
import smbus, math, time

# Kalman filters
kalmanX = KalmanAngle()
kalmanY = KalmanAngle()

# Complementary filter state
compX = 0.0
compY = 0.0

# In-memory log storage
logs = []  # each entry: [timestamp, raw_roll, raw_pitch, kalmanX, kalmanY, compX, compY]

# I2C & MPU‑6050 registers
bus  = smbus.SMBus(1)
ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

radToDeg = 57.2957786

def MPU_Init():
    bus.write_byte_data(ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(ADDR, PWR_MGMT_1, 1)
    bus.write_byte_data(ADDR, CONFIG,    6)
    bus.write_byte_data(ADDR, GYRO_CONFIG,24)
    bus.write_byte_data(ADDR, INT_ENABLE, 1)


def read_raw_data(reg):
    high = bus.read_byte_data(ADDR, reg)
    low  = bus.read_byte_data(ADDR, reg+1)
    val  = (high << 8) | low
    if val > 32767:
        val -= 65536
    return val


def stream_data(socketio):
    """Background task: read MPU, apply filters, emit via socketio."""
    global compX, compY
    MPU_Init()
    time.sleep(1)
    timer = time.time()

    # seed filters with initial accel angles
    ax = read_raw_data(ACCEL_XOUT_H)
    ay = read_raw_data(ACCEL_XOUT_H+2)
    az = read_raw_data(ACCEL_XOUT_H+4)
    raw_roll  = math.atan2(ay, az) * radToDeg
    raw_pitch = math.atan(-ax/math.sqrt(ay*ay + az*az)) * radToDeg
    kalmanX.setAngle(raw_roll)
    kalmanY.setAngle(raw_pitch)
    compX, compY = raw_roll, raw_pitch

    try:
        while True:
            ax = read_raw_data(ACCEL_XOUT_H)
            ay = read_raw_data(ACCEL_XOUT_H+2)
            az = read_raw_data(ACCEL_XOUT_H+4)
            gx = read_raw_data(GYRO_XOUT_H)
            gy = read_raw_data(GYRO_XOUT_H+2)

            dt = time.time() - timer
            timer = time.time()

            # raw accel angles
            raw_roll  = math.atan2(ay, az) * radToDeg
            raw_pitch = math.atan(-ax/math.sqrt(ay*ay + az*az)) * radToDeg

            # gyro rates
            gxr = gx / 131.0
            gyr = gy / 131.0

            # Kalman filter
            kx = kalmanX.getAngle(raw_roll,  gxr, dt)
            ky = kalmanY.getAngle(raw_pitch, gyr, dt)

            # Complementary filter
            compX = 0.93*(compX + gxr*dt) + 0.07*raw_roll
            compY = 0.93*(compY + gyr*dt) + 0.07*raw_pitch

            # log entry
            t = time.time()
            logs.append([t, raw_roll, raw_pitch, kx, ky, compX, compY])

            # emit all filters
            socketio.emit('imu_data', {
                't': t,
                'raw': {'x': raw_roll,  'y': raw_pitch},
                'kalman': {'x': kx, 'y': ky},
                'comp': {'x': compX, 'y': compY}
            })

            socketio.sleep(0.02)
    except Exception as e :
        print(e)