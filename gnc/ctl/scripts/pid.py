import numpy as np

class PID:
    def __init__(self, kp, ki, kd, max_accumulator, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_accumulator = max_accumulator
        self.dt = dt
        self.last_t = None

        self.last_e = 0
        self.acumulator = 0
        self.last_out = 0

    def __call__(self, target, state, v=None, t=None, v_target=0):
        if self.last_t is None or t is None or self.last_t > t:
            dt = self.dt
        elif self.last_t == t:
            return self.last_out
        else:
            dt = t - self.last_t
        self.last_t = t
        e = target - state
        self.acumulator += e * dt
        self.acumulator = np.clip(self.acumulator, -self.max_accumulator, self.max_accumulator)
        p = self.kp * e
        i = self.ki * self.acumulator
        if v is None:
            d = self.kd * (e - self.last_e) / dt
        else:
            d = -self.kd * v
            #print("diff: ", d, self.kd * (e - self.last_e) / dt)
        self.last_e = e
        out = p + i + d
        self.last_out = out
        return out
