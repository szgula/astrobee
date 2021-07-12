import numpy as np

class LQR:
    def __init__(self, K, N):
        self.K = K
        self.N = N
        self.last_out = 0

    def __call__(self, target, state, v, t=None, v_target=0):
        if v is None:
            v = 0
        e1 = target - state
        e2 = v_target - v
        out = self.K[0] * e1 + self.K[1] * e2
        return out
