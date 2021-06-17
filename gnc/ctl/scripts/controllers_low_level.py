import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from pid import PID
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

class OrientationController:
    def __init__(self, dt):
        #self.contr_x  = PID(0.01, 0, 7/4, 0, dt)  # 4,0,7
        self.contr_x  = PID(0.075, 0.0005, 18.0/100, 20, dt)  # 4,0,7
        self.contr_y  = PID(0.075, 0.0005, 18.0/100, 20, dt)
        self.contr_z  = PID(0.075, 0.0005, 18.0/100, 20, dt)
        self.limit = 0.2 * 0.1
    
    def __call__(self, x_target, y_target, z_target, orientation):
        orientation = list(euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w)))
        #print("orientation controller -> target: {:.2f} {:.2f} {:.2f}, orient: {:.2f} {:.2f} {:.2f}".format(*np.rad2deg([x_target, y_target, z_target] + orientation)) )
        torque_x = self.contr_x(x_target, 0)
        torque_y = self.contr_y(y_target, 0)
        torque_z = self.contr_z(z_target, 0)
        torque_x, torque_y, torque_z = np.clip(torque_x, -self.limit, self.limit), np.clip(torque_y, -self.limit, self.limit), np.clip(torque_z, -self.limit, self.limit)
        return [torque_x, torque_y, torque_z]

class PositionController:
    def __init__(self, dt):
        self.contr_x  = PID(0.23, 0.005, 0.9, 20, dt) 
        self.contr_y  = PID(0.23, 0.005, 0.9, 20, dt)
        self.contr_z  = PID(0.23, 0.005, 0.9, 20, dt)
        self.limit = 0.2 
    
    def __call__(self, x_target, y_target, z_target, position):
        force_x = self.contr_x(x_target, 0)
        force_y = self.contr_y(y_target, 0)
        force_z = self.contr_z(z_target, 0)
        force_x, force_y, force_z = np.clip(force_x, -self.limit, self.limit), np.clip(force_y, -self.limit, self.limit), np.clip(force_z, -self.limit, self.limit)
        return [force_x, force_y, force_z]