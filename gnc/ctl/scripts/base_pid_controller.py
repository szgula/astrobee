#!/usr/bin/env python

import rospy
import tf2_ros
from ff_msgs.msg import FamCommand
from ff_msgs.msg import Heartbeat
from ff_msgs.msg import FlightMode
from ff_msgs.msg import EkfState
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time


class BasePIDController:
    def __init__(self):
        rospy.logwarn('Initialise CTL...')
        time.sleep(2)
        self.rate_contr_hz = 62.5
        rospy.init_node('ctl_node', anonymous=False)
        self.pub_ctl = rospy.Publisher(r"gnc/ctl/command", FamCommand, queue_size=5)
        self.pub_heart = rospy.Publisher(r"heartbeat", Heartbeat, queue_size=5, latch=True)
        self.pub_mode = rospy.Publisher(r"/mob/flight_mode", FlightMode, queue_size=5, latch=True)
        self.rate = rospy.Rate(self.rate_contr_hz) # 62.5hz
        #rospy.Subscriber("gnc/ekf", EkfState, self._ekf_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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
        self.initial_pos = [0, 0, 0]

        self.target_position = [0, 0, 0]
        self.target_orientation = [0, 0, 0, 0]

        self.last_torque = [0, 0, 0]
        self.last_force = [0, 0, 0]
        rospy.Subscriber("loc/truth/pose", PoseStamped, self._ekf_callback)


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

    def accuare_target(self):
        try:
            trans = self.tf_buffer.lookup_transform("truth", "target", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #rospy.logwarn('[ctl] get tf transformation exception')
            #self.tf_buffer.clear()
            self.target_position = [0, 0, 0]
            self.target_orientation = [0, 0, 0, 1]
            return
        pos_3dvec = trans.transform.translation
        orient_quat = trans.transform.rotation
        self.target_position = [pos_3dvec.x, pos_3dvec.y, pos_3dvec.z]
        self.target_orientation = [orient_quat.x, orient_quat.y, orient_quat.z, orient_quat.w]
        

    def spin(self):
        rospy.logwarn('[ctl] Starts talking...')
        while not rospy.is_shutdown():
            self.control()
            self.accuare_target()
            if self.n_command % 25 == 0:
                self._health_communication()
            self.rate.sleep()

    def _print_status_informattion(self, dx, dy, dz, x_rot, y_rot, z_rot, torques, forces, enabled=False):
        if enabled:
            rospy.logwarn("step: {:.2f} \t orient: {:.1f} {:.1f} {:.1f} \t pos: {:.2f} {:.2f} {:.2f}, \ttorques*100: {:.2f} {:.2f} {:.2f}, \tforces*10: {:.2f} {:.2f} {:.2f}".format( self.n_command / 100.0, 
                        np.rad2deg(x_rot), np.rad2deg(y_rot), np.rad2deg(z_rot) ,
                        dx, dy, dz,
                        torques[0]*100, torques[1]*100, torques[2]*100,
                        forces[0]*10, forces[1]*10, forces[2]*10))

    def _health_communication(self):
        self._update_heartbeat_mes()
        self.pub_heart.publish(self.mes_heartbeat)
        self._update_flight_mode_mes()
        self.pub_mode.publish(self.mes_flight_mode)
        

    def control(self):
        if self.n_command < 100:
            torques = self.orient_contr(0, 0, 0, self._ekf_state.pose.orientation)
            forces = [0, 0, 0]
            control_mode=1
            status=3
        else:
            dx, dy, dz = self.target_position
            orientation = euler_from_quaternion(self.target_orientation)
            x_rot, y_rot, z_rot = orientation 
            torques = self.orient_contr(x_rot, y_rot, z_rot, self._ekf_state.pose.orientation)
            forces = self.pos_contr(dx, dy, dz, self._ekf_state.pose.position)
            control_mode=2
            status=2
            self._print_status_informattion(dx, dy, dz, x_rot, y_rot, z_rot, torques, forces, enabled=(self.n_command % 25 == 0))

        self._update_command_mes(control_mode=control_mode, status=status, torques=torques, forces=forces)
        self.pub_ctl.publish(self.mes_command)
        self.last_force = forces
        self.last_torque = torques
        

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


def main():
    while True:
        try:
            node = BasePIDController()
            node.spin()
        except rospy.ROSTimeMovedBackwardsException:
            node.tf_buffer.clear()
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")
        except rospy.ROSInterruptException:
            return



if __name__ == '__main__':
    main()
