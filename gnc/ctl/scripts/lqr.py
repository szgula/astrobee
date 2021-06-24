import numpy as np

class LQR:
    def __init__(self, K, N):
        self.K = K
        self.N = N
        self.last_out = 0

    def __call__(self, target, state, v, t=None):
        if v is None:
            v = 0
        e = target - state
        out = target * self.N + (self.K[0] * e - self.K[1] * v)
        return out
