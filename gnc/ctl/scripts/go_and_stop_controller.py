#!/usr/bin/env python

import rospy
from ff_msgs.msg import FamCommand
from ff_msgs.msg import Heartbeat
from ff_msgs.msg import FlightMode
from copy import deepcopy


class StopAngGoController:
    def __init__(self):
        rospy.init_node('ctl_node', anonymous=False)
        self.pub_ctl = rospy.Publisher(r"gnc/ctl/command", FamCommand, queue_size=5)
        self.pub_heart = rospy.Publisher(r"heartbeat", Heartbeat, queue_size=5, latch=True)
        self.pub_mode = rospy.Publisher(r"/mob/flight_mode", FlightMode, queue_size=5, latch=True)
        self.rate = rospy.Rate(62.5) # 62.5hz

        self.n_command = 0
        self.n_heart = 0
        self.n_fl_mode = 0
        self.mes_command = self._get_command_mes()
        self.mes_heartbeat = self._get_heartbeat_mes()
        self.mes_flight_mode = self._get_fligh_mode_mes()

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


    def spin(self):
        print('Starts talking...')
        while not rospy.is_shutdown():
            self.control()
            if self.n_command % 25 == 0:
                self._health_communication()
            self.rate.sleep()

    def _health_communication(self):
        self._update_heartbeat_mes()
        self.pub_heart.publish(self.mes_heartbeat)
        self._update_flight_mode_mes()
        self.pub_mode.publish(self.mes_flight_mode)

    def control(self):
        self._update_command_mes(control_mode=1, status=3)
        if self.n_command > 500:
            self._update_command_mes(control_mode=2, status=2, forces=[-0.5, 0.2, 0])
        self.pub_ctl.publish(self.mes_command)
            
        

if __name__ == '__main__':
    try:
        node = StopAngGoController()
        node.spin()
    except rospy.ROSInterruptException:
        pass
