class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = 0
        self._prev_error = 0
        self._integral = 0

    def __call__(self, measured):
        error = self.setpoint - measured
        self._integral += error
        derivative = error - self._prev_error
        self._prev_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative
