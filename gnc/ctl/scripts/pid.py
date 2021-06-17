import numpy as np

class PID:
    def __init__(self, kp, ki, kd, max_accumulator, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_accumulator = max_accumulator
        self.dt = dt

        self.last_e = 0
        self.acumulator = 0

    def __call__(self, target, state):
        e = target - state
        self.acumulator += e * self.dt
        self.acumulator = np.clip(self.acumulator, -self.max_accumulator, self.max_accumulator)
        p = self.kp * e
        i = self.ki * self.acumulator
        d = self.kd * (e - self.last_e) / self.dt
        self.last_e = e
        return p + i + d
