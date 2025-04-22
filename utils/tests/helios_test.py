import math
import time
import smbus
from servo import set_servo_angle as s

# ====== MPU6050 Configuration ======
MPU6050_ADDR = 0x68
PWR_MGMT_1    = 0x6B
SMPLRT_DIV    = 0x19
CONFIG        = 0x1A
GYRO_CONFIG   = 0x1B
ACCEL_CONFIG  = 0x1C
ACCEL_XOUT_H  = 0x3B
GYRO_XOUT_H   = 0x43

# Create I2C bus
bus = smbus.SMBus(1)  # Bus 1 on Raspberry Pi

# ====== Robot Configuration ======
l1 = 5.0    # upper leg length
l2 = 5.0    # lower leg length
L_max = l1 + l2 * 0.9    # default ground height

# Default hip angle (static stance)
HIP_FIXED = 110  # degrees

# Servo biases\

servo_biases = {
    4: 4.0, 3: 0.0, 2: 0.0,
    7: 0.0, 6: 0.0, 5: 0.0,
    8: 0.0, 9: 0.0, 10:0.0,
    14:0.0,12: 0.0, 11:0.0,
}

# Leg definitions
legs = {
    'front_left':  {'hip':4,  'knee':3,  'calf':2},
    'front_right': {'hip':8,  'knee':9,  'calf':10},
    'hind_right':  {'hip':14, 'knee':12, 'calf':11},
    'hind_left':   {'hip':7,  'knee':6,  'calf':5},
}

# PID gains & targets
PID_GAINS = { 'roll':{'P':2.0,'I':0.01,'D':0.5}, 'pitch':{'P':2.0,'I':0.01,'D':0.5} }
TARGET     = {'roll':0.0, 'pitch':0.0}
MAX_CORR   = {'roll':15.0, 'pitch':15.0}
UPDATE_MS  = 50

# ====== MPU6050 Helpers ======
def initialize_mpu():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU6050_ADDR, CONFIG,     0)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG,0)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG,0)


def read_raw(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low  = bus.read_byte_data(MPU6050_ADDR, addr+1)
    val  = (high<<8) | low
    return val-65536 if val>32767 else val


def get_imu():
    ax = read_raw(ACCEL_XOUT_H)/16384.0
    ay = read_raw(ACCEL_XOUT_H+2)/16384.0
    az = read_raw(ACCEL_XOUT_H+4)/16384.0
    gx = read_raw(GYRO_XOUT_H)/131.0
    gy = read_raw(GYRO_XOUT_H+2)/131.0
    gz = read_raw(GYRO_XOUT_H+4)/131.0
    roll  = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay+az*az)))
    return {'roll':roll, 'pitch':pitch, 'acc':(ax,ay,az), 'gyro':(gx,gy,gz)}

# ====== IK ======
def compute_IK(L, l1, l2):
    L = min(max(L, abs(l1-l2)+0.1), l1+l2)
    theta = math.degrees(math.acos((l1*l1 + l2*l2 - L*L)/(2*l1*l2)))
    knee = math.degrees(math.atan2(l2*math.sin(math.radians(theta)), l1 + l2*math.cos(math.radians(theta))))
    calf = 180 - theta
    return knee, calf

# ====== PID ======
class PID:
    def __init__(self, P,I,D,target,limits):
        self.Kp, self.Ki, self.Kd = P,I,D
        self.target=target; self.limits=limits
        self.prev=0; self.int=0; self.last=time.time()
    def compute(self, val):
        now = time.time(); dt = now-self.last if now>self.last else 1e-3; self.last=now
        err = self.target - val
        P = self.Kp*err
        self.int = max(min(self.int+err*dt, self.limits[1]/self.Ki), self.limits[0]/self.Ki)
        I = self.Ki*self.int
        D = -self.Kd*((val-self.prev)/dt)
        out = max(min(P+I+D, self.limits[1]), self.limits[0])
        self.prev=val; return out

# ====== Main Stand Test ======
def main():
    initialize_mpu()
    print("Calibrating... keep robot still")
    time.sleep(1)
    init = get_imu()
    TARGET['roll'], TARGET['pitch'] = init['roll'], init['pitch']
    print(f"Calibrated roll={TARGET['roll']:.2f}, pitch={TARGET['pitch']:.2f}")

    roll_pid  = PID(**PID_GAINS['roll'],  target=TARGET['roll'],  limits=(-MAX_CORR['roll'], MAX_CORR['roll']))
    pitch_pid = PID(**PID_GAINS['pitch'], target=TARGET['pitch'], limits=(-MAX_CORR['pitch'],MAX_CORR['pitch']))

    # Set static stance
    for leg, p in legs.items():
        knee, calf = compute_IK(L_max, l1, l2)
        s(p['hip'],   HIP_FIXED + servo_biases.get(p['hip'],0))
        s(p['knee'],  knee      + servo_biases.get(p['knee'],0))
        s(p['calf'],  calf      + servo_biases.get(p['calf'],0))
    time.sleep(0.5)

    print("Starting PID stabilization test. Ctrl+C to stop.")
    try:
        while True:
            imu = get_imu()
            rc = roll_pid.compute( imu['roll'] )
            pc = pitch_pid.compute(imu['pitch'])
            # Apply corrections: adjust knee angles only
            for leg, p in legs.items():
                knee, calf = compute_IK(L_max + (-pc if 'front' in leg else pc)
                                       + (-rc if 'left' in leg else rc), l1, l2)
                s(p['knee'], knee + servo_biases.get(p['knee'],0))
            time.sleep(UPDATE_MS/1000.0)
    except KeyboardInterrupt:
        print("Stand stabilization stopped.")

if __name__ == '__main__':
    main()
