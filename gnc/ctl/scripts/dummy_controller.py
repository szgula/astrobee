#!/usr/bin/env python

import rospy
from ff_msgs.msg import FamCommand
from ff_msgs.msg import Heartbeat
from ff_msgs.msg import FlightMode
from copy import deepcopy


def talker():
    n = 0
    n_heart = 0
    rospy.init_node('ctl_node', anonymous=False)
    pub = rospy.Publisher(r"gnc/ctl/command", FamCommand, queue_size=5)
    pub_heart = rospy.Publisher(r"heartbeat", Heartbeat, queue_size=5, latch=True)
    pub_mode = rospy.Publisher(r"/mob/flight_mode", FlightMode, queue_size=5, latch=True)
    rate = rospy.Rate(62.5) # 62.5hz
    print('Starts talking...')
    while not rospy.is_shutdown():
        mes = FamCommand()
        mes.header.seq = n
        mes.header.frame_id = "body"
        n += 1
        now = rospy.get_rostime()
        mes.header.stamp.secs = now.secs
        mes.header.stamp.nsecs = now.nsecs
        mes.control_mode = 1
        mes.status = 3
        if n > 500:
            mes.control_mode = 2
            mes.status = 2
            mes.wrench.force.x = -0.5
            #mes.accel.x = mes.wrench.force.x / 9.58
            #mes.position_error.x = 0.01
            #mes.position_error.y = 0.0001
            #mes.position_error.z = 0.0001
            #mes.attitude_error_mag = 0.0210811849684
            #mes.attitude_error.x = -0.00151798105799
            #mes.attitude_error.y = 0.000572398828808
            #mes.attitude_error.z = -0.0104213329032
        pub.publish(mes)
        if n % 25 == 0:
            mes_heart = Heartbeat()
            mes_heart.header = deepcopy(mes.header)
            mes_heart.header.seq = n_heart
            #print("message header", mes_heart.header)
            #mes_heart.nodelet_manager= "/llp_gnc_ctl"
            mes_heart.nodelet_manager= "/llp_gnc"
            mes_heart.node = "ctl"
            pub_heart.publish(mes_heart)
            n_heart += 1
            mes_mode = FlightMode()
            mes_mode.header = deepcopy(mes.header)
            mes_mode.name = "nominal"
            mes_mode.control_enabled = True
            mes_mode.speed = 2
            pub_mode.publish(mes_mode)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
