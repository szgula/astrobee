#!/usr/bin/env python

import rospy
from ff_msgs.msg import FamCommand
from ff_msgs.msg import Heartbeat
from ff_msgs.msg import FlightMode
from ff_msgs.msg import EkfState
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class StopAngGoController:
    def __init__(self):
        self.rate_contr_hz = 62.5
        rospy.init_node('ctl_node', anonymous=False)
        self.pub_ctl = rospy.Publisher(r"gnc/ctl/command", FamCommand, queue_size=5)
        self.pub_heart = rospy.Publisher(r"heartbeat", Heartbeat, queue_size=5, latch=True)
        self.pub_mode = rospy.Publisher(r"/mob/flight_mode", FlightMode, queue_size=5, latch=True)
        self.rate = rospy.Rate(self.rate_contr_hz) # 62.5hz
        #rospy.Subscriber("gnc/ekf", EkfState, self._ekf_callback)
        rospy.Subscriber("loc/truth/pose", PoseStamped, self._ekf_callback)

        self.n_command = 0
        self.n_heart = 0
        self.n_fl_mode = 0
        self.mes_command = self._get_command_mes()
        self.mes_heartbeat = self._get_heartbeat_mes()
        self.mes_flight_mode = self._get_fligh_mode_mes()
        self._ekf_state = EkfState()

        self.orient_contr = OrientationController(1/self.rate_contr_hz)
        self.pos_contr = PositionController(1/self.rate_contr_hz)
        self.orientation_read = False
        self.position_read = False
        self.initial_pos = 0


    def _get_heartbeat_mes(self):
        mes_heart = Heartbeat()
        now = rospy.get_rostime()
        mes_heart.header.stamp.secs = now.secs
        mes_heart.header.stamp.nsecs = now.nsecs
        mes_heart.header.seq = self.n_command
        mes_heart.nodelet_manager= "/llp_gnc"
        mes_heart.node = "ctl"
        return mes_heart
    
    def _get_command_mes(self):
        mes_command = FamCommand()
        mes_command.header.seq = 0
        mes_command.header.frame_id = "body"
        now = rospy.get_rostime()
        mes_command.header.stamp.secs = now.secs
        mes_command.header.stamp.nsecs = now.nsecs
        return mes_command

    def _get_fligh_mode_mes(self):
        mes_mode = FlightMode()
        now = rospy.get_rostime()
        mes_mode.header.stamp.secs = now.secs
        mes_mode.header.stamp.nsecs = now.nsecs
        mes_mode.name = "nominal"
        mes_mode.control_enabled = True
        mes_mode.speed = 2     
        return mes_mode

    def _update_command_mes(self, control_mode=1, status=3, forces=[0,0,0], torques=[0,0,0]):
        self.n_command += 1
        now = rospy.get_rostime()
        self.mes_command.header.seq = self.n_command
        self.mes_command.header.stamp.secs = now.secs
        self.mes_command.header.stamp.nsecs = now.nsecs
        self.mes_command.control_mode = control_mode
        self.mes_command.status = status
        self.mes_command.wrench.force.x = forces[0]
        self.mes_command.wrench.force.y = forces[1]
        self.mes_command.wrench.force.z = forces[2]   
        self.mes_command.wrench.torque.x = torques[0]   
        self.mes_command.wrench.torque.y = torques[1]   
        self.mes_command.wrench.torque.z = torques[2]  

    def _update_heartbeat_mes(self):
        self.n_heart += 1
        now = rospy.get_rostime()
        self.mes_heartbeat.header.stamp.secs = now.secs
        self.mes_heartbeat.header.stamp.nsecs = now.nsecs
        self.mes_heartbeat.header.seq = self.n_heart
    
    def _update_flight_mode_mes(self):
        self.n_fl_mode += 1
        now = rospy.get_rostime()
        self.mes_flight_mode.header.stamp.secs = now.secs
        self.mes_flight_mode.header.stamp.nsecs = now.nsecs
        self.mes_flight_mode.header.seq = self.n_fl_mode

    def _ekf_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        #print('received message with: ', data.pose.position.x, data.pose.orientation.x)
        self._ekf_state = data
        if self.position_read == False:
            self.initial_pos = self._ekf_state.pose.position
        self.orientation_read = True
        self.position_read = True

    def spin(self):
        print('Starts talking...')
        while not rospy.is_shutdown():
            self.control()
            if self.n_command % 25 == 0:
                self._health_communication()
            if self.n_command % 25 == 0:
                if self.orientation_read:
                    orientation = self._ekf_state.pose.orientation
                    orientation = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
                    position = [self._ekf_state.pose.position.x - self.initial_pos.x, 
                                self._ekf_state.pose.position.y - self.initial_pos.y, 
                                self._ekf_state.pose.position.z - self.initial_pos.z]
                    print("step: ", self.n_command / 100.0, " orientation: ", position)
                else:
                    print("step: ", self.n_command / 100.0)
            self.rate.sleep()

    def _health_communication(self):
        self._update_heartbeat_mes()
        self.pub_heart.publish(self.mes_heartbeat)
        self._update_flight_mode_mes()
        self.pub_mode.publish(self.mes_flight_mode)
        

    def control(self):
        trajectory = [  #(1000, 0, 0, 0), 
                        (2000, (-0.8, 0, 0),     (np.pi/3, 0, 0)), 
                        (3000, (-0.8, 0.8, 0),   (0, 0, 0)),
                        (4000, (-0.8, 0.8, 0.8), (0, np.pi/3, 0)),
                        (5000, (0, 0.8, 0.8),    (0, 0, 0)),
                        (6000, (0, 0, 0.8),      (0, 0, np.pi/3)),
                        (7000, (0, 0, 0.8),      (0, 0, 0))]
        if self.n_command < 100:
            torques = self.orient_contr(0, 0, 0, self._ekf_state.pose.orientation)
            forces = [0, 0, 0]
            control_mode=1
            status=3
        else:
            if self.n_command // 1000 < len(trajectory):
                untill_t, d_pos, orient_ = trajectory[self.n_command // 1000]
                dx, dy, dz = d_pos
                x_rot, y_rot, z_rot = orient_
            else:
                dx, dy, dz = 0, 0, 0
                x_rot, y_rot, z_rot = 0, 0, 0
            torques = self.orient_contr(x_rot, y_rot, z_rot, self._ekf_state.pose.orientation)
            #forces = self.pos_contr(self.initial_pos.x + dx, self.initial_pos.y + dy, self.initial_pos.z + dz, self._ekf_state.pose.position)
            forces = [0, 0, 0]
            control_mode=2
            status=2

        self._update_command_mes(control_mode=control_mode, status=status, torques=torques, forces=forces)
        self.pub_ctl.publish(self.mes_command)
        


class OrientationController:
    def __init__(self, dt):
        #self.contr_x  = PID(0.01, 0, 7/4, 0, dt)  # 4,0,7
        self.contr_x  = PID(1, 0, 7/3, 0, dt)  # 4,0,7
        self.contr_y  = PID(1, 0, 7/3, 0, dt)
        self.contr_z  = PID(1, 0, 7/3, 0, dt)
    
    def __call__(self, x_target, y_target, z_target, orientation):
        orientation = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        torque_x = self.contr_x(x_target, orientation[0])
        torque_y = self.contr_y(y_target, orientation[1])
        torque_z = self.contr_z(z_target, orientation[2])
        return [torque_x, torque_y, torque_z]

class PositionController:
    def __init__(self, dt):
        self.contr_x  = PID(1, 0, 4, 0, dt) 
        self.contr_y  = PID(0.3, 0, 1.2, 0, dt)
        self.contr_z  = PID(0.3, 0, 1.2, 0, dt)
    
    def __call__(self, x_target, y_target, z_target, position):
        torque_x = self.contr_x(x_target, position.x)
        torque_y = self.contr_y(y_target, position.y)
        torque_z = self.contr_z(z_target, position.z)
        return [torque_x, torque_y, torque_z]

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



if __name__ == '__main__':
    try:
        node = StopAngGoController()
        node.spin()
    except rospy.ROSInterruptException:
        pass
