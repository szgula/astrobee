import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from pid import PID
from lqr import LQR
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

class OrientationController:
    def __init__(self, dt):
        #self.contr_x  = PID(0.075, 0.0005, 18.0/100, 20, dt)  # 0.075, 0.0005, 18.0/100, 20, dt
        #self.contr_y  = PID(0.075, 0.0005, 18.0/100, 20, dt)  # 0.15, 0.015, 30.0/100, 200, dt
        #self.contr_z  = PID(0.075, 0.0005, 18.0/100, 20, dt)
        self.contr_x = LQR([0.1393, 0.2629]) # 0.0443    0.1796
        self.contr_y = LQR([0.1393, 0.2629])
        self.contr_z = LQR([0.1393, 0.2629])
        self.limit = 0.2 * 0.1
    
    def __call__(self, x_target, y_target, z_target, orientation, last_vel=None, t=None, target_vel=[0,0,0], inertia_change_param=0):
        orientation = list(euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w)))
        #print("orientation controller -> target: {:.2f} {:.2f} {:.2f}, orient: {:.2f} {:.2f} {:.2f}".format(*np.rad2deg([x_target, y_target, z_target] + orientation)) )
        vx, vy, vz = None, None, None
        if last_vel is not None:
            vx, vy, vz = last_vel.twist.angular.x, last_vel.twist.angular.y, last_vel.twist.angular.z
        torque_x = self.contr_x(x_target, 0, vx, t, target_vel[0], inertia_change_indicator=inertia_change_param)
        torque_y = self.contr_y(y_target, 0, vy, t, target_vel[1], inertia_change_indicator=inertia_change_param)
        torque_z = self.contr_z(z_target, 0, vz, t, target_vel[2], inertia_change_indicator=inertia_change_param)
        #if abs(z_target) > np.deg2rad(5):
        #    torque_z = 1
        torque_x, torque_y, torque_z = np.clip(torque_x, -self.limit, self.limit), np.clip(torque_y, -self.limit, self.limit), np.clip(torque_z, -self.limit, self.limit)
        return [torque_x, torque_y, torque_z]

class PositionController:
    def __init__(self, dt):
        self.limit = 0.2 
        #self.contr_x  = PID(0.35, 0.002, 0.82, 200, dt) 
        #self.contr_y  = PID(0.35, 0.002, 0.82, 200, dt)
        #self.contr_z  = PID(0.35, 0.002, 0.82, 200, dt) # 0.35, 0.02, 0.62, 200, dt
        self.contr_x = LQR([0.6296, 2.7756])  # [0.1413, -0.5845], 0.1416
        self.contr_y = LQR([0.6296, 2.7756])  #([0.6296, 2.5756], 0.6303) was the best so far
        self.contr_z = LQR([0.6296, 2.7756])
        
    
    def __call__(self, x_target, y_target, z_target, position, last_vel=None, t=None, target_vel=[0,0,0]):
        vx, vy, vz = None, None, None
        if last_vel is not None:
            vx, vy, vz = last_vel.twist.linear.x, last_vel.twist.linear.y, last_vel.twist.linear.z
        force_x = self.contr_x(x_target, 0, vx, t, target_vel[0])
        force_y = self.contr_y(y_target, 0, vy, t, target_vel[1])
        force_z = self.contr_z(z_target, 0, vz, t, target_vel[2])
        force_x, force_y, force_z = np.clip(force_x, -self.limit, self.limit), np.clip(force_y, -self.limit, self.limit), np.clip(force_z, -self.limit, self.limit)
        #if abs(force_x) > self.limit or abs(force_y) > self.limit or abs(force_z) > self.limit:
        #    rospy.logwarn(" target rot {:.2f} {:.2f} {:.2f}".format(np.rad2deg(x_target), np.rad2deg(y_target), np.rad2deg(z_target)))
        #    rospy.logwarn(" time {:.5f} {:.5f} {:.5f}".format(t, self.contr_x.last_t, ))
        #    if last_vel is not None:
        #        rospy.logwarn(" target rot {:.2f} {:.2f} {:.2f}".format(np.rad2deg(vx), np.rad2deg(vy), np.rad2deg(vz)))
        return [force_x, force_y, force_z]