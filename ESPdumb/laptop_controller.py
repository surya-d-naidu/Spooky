import socket
import time

ESP_IP = '192.168.4.1'  # Change to ESP32 IP
ESP_PORT = 4210

# Servo channel mapping for each leg
LEGS = {
    'front_left':  {'hip': 4,  'knee': 3,  'calf': 2},
    'front_right': {'hip': 8,  'knee': 9,  'calf': 10},
    'hind_right':  {'hip': 14, 'knee': 12, 'calf': 11},
    'hind_left':   {'hip': 7,  'knee': 6,  'calf': 5},
}
LEG_SEQUENCE = ['front_left', 'hind_right', 'front_right', 'hind_left']  # Diagonal crawl

# Gait parameters
HIP_FIXED = 110
KNEE_DOWN = 60
KNEE_UP = 30
CALF_DOWN = 60
CALF_UP = 30
STEP_DELAY = 0.25


def send_servo(sock, channel, angle):
    msg = f"{channel} {angle}\n".encode()
    sock.sendto(msg, (ESP_IP, ESP_PORT))
    time.sleep(0.01)

def set_leg(sock, leg, hip, knee, calf):
    send_servo(sock, LEGS[leg]['hip'], hip)
    send_servo(sock, LEGS[leg]['knee'], knee)
    send_servo(sock, LEGS[leg]['calf'], calf)

def all_legs_down(sock):
    for leg in LEGS:
        set_leg(sock, leg, HIP_FIXED, KNEE_DOWN, CALF_DOWN)

def move_leg(sock, leg, up=True):
    # up=True: lift, up=False: down
    knee = KNEE_UP if up else KNEE_DOWN
    calf = CALF_UP if up else CALF_DOWN
    set_leg(sock, leg, HIP_FIXED, knee, calf)

def crawl_step(sock, sequence, direction=1):
    # direction=1: forward, -1: backward
    for i in range(len(sequence)):
        leg = sequence[i] if direction==1 else sequence[-i-1]
        # 1. Lift leg
        move_leg(sock, leg, up=True)
        time.sleep(STEP_DELAY)
        # 2. Lower leg
        move_leg(sock, leg, up=False)
        time.sleep(STEP_DELAY)

def rotate(sock, left=True):
    # For rotation, swing diagonal legs
    seq = ['front_left', 'hind_right'] if left else ['front_right', 'hind_left']
    for leg in seq:
        move_leg(sock, leg, up=True)
        time.sleep(STEP_DELAY)
        move_leg(sock, leg, up=False)
        time.sleep(STEP_DELAY)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    all_legs_down(sock)
    print("Controls: w=forward, s=back, a=left, d=right, q=quit")
    while True:
        cmd = input('> ').strip().lower()
        if cmd == 'w':
            crawl_step(sock, LEG_SEQUENCE, direction=1)
        elif cmd == 's':
            crawl_step(sock, LEG_SEQUENCE, direction=-1)
        elif cmd == 'a':
            rotate(sock, left=True)
        elif cmd == 'd':
            rotate(sock, left=False)
        elif cmd == 'q':
            break
        else:
            print("w/s/a/d/q only")
    sock.close()

if __name__ == '__main__':
    main()
