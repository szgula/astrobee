import numpy as np

class LQR:
    def __init__(self, K, inertia_change_scalar=0.57):
        self.K = K
        self.last_out = 0
        self.inertia_change_scalar = inertia_change_scalar

    def __call__(self, target, state, v, t=None, v_target=0, inertia_change_indicator=0):
        if v is None:
            v = 0
        e1 = target - state
        e2 = v_target - v
        K0 = self.K[0] #+ self.K[0] * inertia_change_indicator * self.inertia_change_scalar
        K1 = self.K[1] + self.K[1] * inertia_change_indicator * self.inertia_change_scalar
        out = K0 * e1 + K1 * e2
        return out