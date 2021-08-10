import numpy as np

class Model:
    def __init__(self, A, B, C, D, E=[[0,0], [0,0]], dt=1/62.5, m=None):
        self.A = np.array(A)
        self.B = np.array(B)
        self.C = np.array(C)
        self.D = np.array(D)
        self.E = np.array(E)
        self.dt = dt
        self.m = m
        self.x = np.array([[0], [0]])

    def step(self, u, dt=None):
        if dt is None:
            #dt = self.dt
            self.A[0, 1] = self.dt
            self.B[1, 0] = self.dt / self.m
        else:
            self.A[0, 1] = dt
            self.B[1, 0] = dt / self.m
        x_next = np.dot(self.A, self.x) + np.dot(self.B, u) + np.dot(self.E, self.x*abs(self.x))
        y = np.dot(self.C, x_next)
        #print(y)
        #print(x_next)
        self.x = x_next
        return y[0, 0], x_next

def get_pos_model(m=9.087, v_next=0.99664271, drag=0.00691741):
    m = m #9.087
    dt = 1.0 / 62.5
    #A = [[0., 1.], [0., 0.]]
    #B = [[0.], [1.0 / m]]
    #C = [[1., 0.]]
    #D = 0

    dA = [[1, dt], [0, v_next]]
    dB = [[0.], [dt / m]]
    dC = [[1., 0.]]
    dD = 0
    #drag_ = -0.5 * 1.05 * 0.092903 * 1.225
    dE = [[0., 0.], [0, drag]]
    model = Model(dA, dB, dC, dD, dE, dt, m)
    return model