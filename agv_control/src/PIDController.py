#! /usr/bin/env python3

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

    def reset(self):
        self.last_error = 0
        self.integral = 0