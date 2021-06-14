#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped 
import rosbag

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import geometry_msgs.msg
from ff_msgs.msg import FamCommand
from ff_hw_msgs.msg import PmcCommand

import numpy as np
from time import strftime
import os
from copy import deepcopy

class CtlTestManager(object):

    def __init__(self, log_path=r'/ros_logs/'):
        # Test cases definition
        self.tests = [
            SimpleRotationTest(),
            SimpleTranslationTest(),
            TranslationWithRotationTest()]
        
        # Internal properties & state & data
        self.rate_contr_hz = 62.5
        self.f_initial_state_acquired = False
        temp_mes = PoseStamped()
        self.initial_pos = temp_mes.pose.position
        self.initial_rot = temp_mes.pose.orientation
        self.last_pos = None
        self.last_vel = None
        self.last_fam = None
        self.last_pmc = None  # propulsion controller
        self.last_target_pos = None
        self.step = 0

        # Loging
        self.log_folder = os.getenv('HOME') + log_path + strftime("%Y%m%d_%H%M")
        self.prepare_log_folders()
        self.ros_bag = None

        # ROS stuff (node, communicattion, transormation)
        rospy.init_node('ctl_test_manager', anonymous=False)
        self.rate = rospy.Rate(self.rate_contr_hz) # 62.5hz
        self.br = tf2_ros.TransformBroadcaster()
        self.msg = geometry_msgs.msg.TransformStamped()
        self.msg.header.frame_id = "world"
        self.msg.child_frame_id = "target"
        rospy.Subscriber("loc/truth/pose", PoseStamped, self._get_astrobee_state)
        rospy.Subscriber("loc/truth/twist", TwistStamped, self._get_astrobee_vel)
        rospy.Subscriber("gnc/ctl/command", FamCommand, self._get_astrobee_fam)
        rospy.Subscriber("hw/pmc/command", PmcCommand, self._get_astrobee_pmc)

    def _get_astrobee_state(self, data):
        pos = data.pose.position
        rot = data.pose.orientation
        self.last_pos = data
        if not self.f_initial_state_acquired:
            self.f_initial_state_acquired = True
            self.initial_pos = pos
            self.initial_rot = rot
            rospy.loginfo("Initialised CTL")

    def _get_astrobee_vel(self, data):
        self.last_vel = data
    
    def _get_astrobee_fam(self, data):
        self.last_fam = data

    def _get_astrobee_pmc(self, data):
        self.last_pmc = data

    def generate_and_send_tf_target(self):
        if self.f_initial_state_acquired:
            target_pos, target_orient = self.get_target()
            self.msg.header.stamp = rospy.Time.now()
            self.msg.transform.translation.x = self.initial_pos.x - target_pos[0]
            self.msg.transform.translation.y = self.initial_pos.y - target_pos[1]
            self.msg.transform.translation.z = self.initial_pos.z - target_pos[2]
            self.msg.transform.rotation.x = target_orient[0]
            self.msg.transform.rotation.y = target_orient[1]
            self.msg.transform.rotation.z = target_orient[2]
            self.msg.transform.rotation.w = target_orient[3]
            self.br.sendTransform(self.msg)

            self.last_target_pos = deepcopy(self.last_pos)
            self.last_target_pos.pose.position.x = self.msg.transform.translation.x
            self.last_target_pos.pose.position.y = self.msg.transform.translation.y
            self.last_target_pos.pose.position.z = self.msg.transform.translation.z
            self.last_target_pos.pose.orientation.x = self.msg.transform.rotation.x
            self.last_target_pos.pose.orientation.y = self.msg.transform.rotation.y
            self.last_target_pos.pose.orientation.z = self.msg.transform.rotation.z
            self.last_target_pos.pose.orientation.w = self.msg.transform.rotation.w


    def spin(self):
        while True:
            self.generate_and_send_tf_target()
            self.log_test_case()
            self.step += 1
            self.rate.sleep()
        #rospy.spin()

    def get_target(self):
        current_test = 1
        if not self.tests[current_test].done:
            if self.ros_bag is None:
                self.init_new_ros_bag(self.tests[current_test].get_test_name())
            pos, orient_quat = self.tests[current_test].get_target_state()
            type_ = "test {}".format(current_test)
            step = self.tests[current_test].step
        else:
            type_ = "default"
            self.close_ros_bag()
            pos, orient_quat = self.get_default_pose()
            step = 0
        if self.step % 100 == 0:
            rospy.loginfo("running {}, step: {} orientation: {:.1f} {:.1f} {:.1f} {:.1f} \t position: {:.2f} {:.2f} {:.2f}".format(
                    type_, step,
                    orient_quat[0], orient_quat[1], orient_quat[2], orient_quat[3],
                    pos[0], pos[1], pos[2]
                ))
        return pos, orient_quat

    def get_default_pose(self):
        """ returns pos and rot in msg format """
        pos, orient_quat = [0, 0, 0], [self.initial_rot.x, self.initial_rot.y, self.initial_rot.z, self.initial_rot.w]
        return pos, orient_quat

    def prepare_log_folders(self):
        # create a log folder, description.txt, 
        if not os.path.isfile(self.log_folder):
            os.mkdir(self.log_folder)
        open(self.log_folder + '/description.txt', 'a').close()

    def log_test_case(self):
        if self.ros_bag is not None:
            if self.last_pos is not None:
                self.ros_bag.write('loc/truth/pose', self.last_pos)
            if self.last_vel is not None:
                self.ros_bag.write('loc/truth/twist', self.last_vel)
            if self.last_fam is not None:
                self.ros_bag.write('gnc/ctl/command', self.last_fam)
            if self.last_pmc is not None:
                self.ros_bag.write('hw/pmc/command', self.last_pmc)
            if self.last_target_pos is not None:
                self.ros_bag.write('loc/reference', self.last_target_pos)
            
        # 1) reference
        # 2) pos
        # 3) velocities
        # 4) effort
        # 5) forces + torques
        pass
    
    def init_new_ros_bag(self, name):
        if self.ros_bag is not None:
            self.close_ros_bag()
        file_path = self.log_folder + "/" + name + ".bag"
        self.ros_bag = rosbag.Bag(file_path, 'w')

    def close_ros_bag(self):
        if self.ros_bag is not None:
            self.ros_bag.close()
            self.ros_bag = None
    
    def __del__(self):  # This is dirty - but context manager seems difficult to be integrated
        if self.ros_bag is not None:
            self.ros_bag.close()





# This should be     
class TestCaseClass(object):
    def __init__(self):
        self.step = 0
        self.done = False
        self.test_case_name = None   # This needs to be set in the child class -> TODO: in future create a abstract set_method 

    def get_target_state(self):
        current_target = [r for r in self.actions_sequence if (self.step in r)][0]
        #current_target = current_target[0]
        pos = current_target.pos
        orientation_quat =  current_target.get_quat()
        self.step += 1
        if self.step >= self.length:
            self.done = True
        return pos, orientation_quat

    def get_test_name(self):
        return self.test_case_name
    
    
class SimpleRotationTest(TestCaseClass):
    def __init__(self, value=66):
        super(SimpleRotationTest, self).__init__()
        self.rotation_value = value
        self.test_case_name = "Simple_rotation_{}deg".format(value)
        self.actions_sequence = [
            Action(   0,  999, (0, 0, 0), (0, 0 , 0)),
            Action(1000, 1999, (0, 0, 0), (value, 0 , 0)),
            Action(2000, 2999, (0, 0, 0), (0, 0 , 0)),
            Action(3000, 3999, (0, 0, 0), (0, value , 0)),
            Action(4000, 4999, (0, 0, 0), (0, 0 , 0)),
            Action(5000, 5999, (0, 0, 0), (0, 0 , value)),
            Action(6000, 6999, (0, 0, 0), (0, 0 , 0)),
        ]
        self.length = 6999

class SimpleTranslationTest(TestCaseClass):
    def __init__(self, value=0.4):
        super(SimpleTranslationTest, self).__init__()
        self.test_case_name = "Simple_traslation_{}cm".format(int(value*100))
        self.actions_sequence = [
            Action(   0,  999, (0, 0, 0), (0, 0 , 0)),
            Action(1000, 1999, (0, value, 0), (0, 0 , 0)),
            Action(2000, 2999, (0, value, value), (0, 0 , 0)),
            Action(3000, 3999, (value, value, value), (0, 0 , 0)),
            Action(4000, 4999, (value, 0, value), (0, 0 , 0)),
            Action(5000, 5999, (value, 0, 0), (0, 0 , 0)),
            Action(6000, 6999, (0, 0, 0), (0, 0 , 0)),
        ]
        self.length = 6999

class TranslationWithRotationTest(TestCaseClass):
    def __init__(self):
        super(TranslationWithRotationTest, self).__init__()
        self.test_case_name = "Translation_with_rotation"
        self.actions_sequence = [
            Action(   0,  999, (0, 0, 0),       (0, 0 , 0)),
            Action(1000, 1999, (0, 0, 0),       (0, 0 , 70)),
            Action(2000, 2999, (0, 0.4, 0),     (0, 0 , 70)),
            Action(3000, 3999, (0, 0.4, 0),     (45, 0 , 70)),
            Action(4000, 4999, (0.4, 0.4, 0),   (45, 0 , 70)),
            Action(5000, 5999, (0.4, 0.4, 0.4), (0, 0 , 70)),
            Action(6000, 6999, (0, 0, 0),       (0, 0 , 0)),
        ]
        self.length = 6999
        

    
class Action(object):
    def __init__(self, t_beg, t_end, pos, rot):
        self.t_0 = t_beg
        self.t_end = t_end
        self.pos = pos
        self.rot = rot
    def __contains__(self, t_):
        return self.t_0 <= t_ and self.t_end >= t_
    def get_quat(self):
        """return list [x, y, z, w]"""
        rot_rad = [np.deg2rad(a_deg) for a_deg in self.rot]  # How about np.deg2rad(np.clip(a_deg, -89, 89)) ?
        return quaternion_from_euler(*rot_rad)


if __name__ == '__main__':
    try:
        node = CtlTestManager()
        node.spin()
    except rospy.ROSInterruptException:
        pass