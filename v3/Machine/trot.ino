#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  120  // Min pulse length out of 4096
#define SERVOMAX  600  // Max pulse length out of 4096

// Servo channel mapping (adjust as needed)
struct Leg {
  uint8_t hip, knee, calf;
  int phase;
};
Leg legs[4] = {
  {4, 3, 2, 0},      // front_left
  {8, 9, 10, -500},  // front_right
  {14, 12, 11, -1000}, // hind_right
  {7, 6, 5, -1500}   // hind_left
};

const float l1 = 5.0; // upper leg length
const float l2 = 5.0; // lower leg length
const float L_max = l1 + l2 * 0.9;
const float L_min = L_max * 0.7;
const int T = 2000; // ms, gait period
const int update_interval_ms = 20;
const int HIP_FIXED = 110;
const float step_length = 15.0;

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);
  // Initial stance
  for (int i = 0; i < 4; i++) {
    float knee, calf;
    computeIK(L_max, l1, l2, knee, calf);
    setServoAngle(legs[i].hip, HIP_FIXED);
    setServoAngle(legs[i].knee, knee);
    setServoAngle(legs[i].calf, calf);
  }
}

void loop() {
  static unsigned long start_time = millis();
  static int prev_active_leg = -1;
  unsigned long t_ms = (millis() - start_time);

  int active_leg = -1;

  for (int i = 0; i < 4; i++) {
    float norm = ((t_ms + legs[i].phase + T) % T) / (float)T;
    float L_des, off, knee_ang, calf_ang, mod_knee;

    // Gait phase logic
    if (norm >= 0.15 && norm < 0.25) {
      float prog = (norm - 0.15) / 0.1;
      L_des = L_max - prog * (L_max - L_min);
      active_leg = i;
    } else if (norm >= 0.25 && norm < 0.35) {
      float prog = (norm - 0.25) / 0.1;
      L_des = L_min + prog * (L_max - L_min);
      active_leg = i;
    } else {
      L_des = L_max;
    }

    computeIK(L_des, l1, l2, knee_ang, calf_ang);

    // Horizontal foot trajectory
    if (norm < 0.15) {
      off = -step_length * (norm / 0.15);
    } else if (norm < 0.35) {
      off = -step_length * 0.2 + step_length * ((norm - 0.15) / 0.2);
    } else {
      off = step_length * 0.8 * (1 - (norm - 0.35) / 0.65);
    }
    mod_knee = knee_ang - off;

    setServoAngle(legs[i].hip, HIP_FIXED);
    // Special bias for front-left knee (channel 3)
    if (legs[i].knee == 3) {
      float a = max(0.0, mod_knee - 20.0);
      setServoAngle(legs[i].knee, a);
    } else {
      setServoAngle(legs[i].knee, mod_knee);
    }
    setServoAngle(legs[i].calf, calf_ang);
  }

  // Optional: body shift logic can be added here if desired

  delay(update_interval_ms);
}

// --- Helper functions ---

void setServoAngle(uint8_t channel, float angle) {
  // Map angle (0-180) to pulse (SERVOMIN-SERVOMAX)
  int pulse = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

// Inverse Kinematics for knee/calf
void computeIK(float L, float l1, float l2, float &knee_angle, float &calf_angle) {
  if (L > (l1 + l2)) L = l1 + l2;
  if (L < fabs(l1 - l2) + 0.1) L = fabs(l1 - l2) + 0.1;
  float theta = 180.0 - (acosf((l1*l1 + l2*l2 - L*L) / (2*l1*l2)) * 180.0 / PI);
  knee_angle = atan2f(l2 * sinf(theta * PI / 180.0), l1 + l2 * cosf(theta * PI / 180.0)) * 180.0 / PI;
  calf_angle = theta;
}