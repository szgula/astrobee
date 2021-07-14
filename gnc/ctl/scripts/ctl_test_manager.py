#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped 
import rosbag

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import geometry_msgs.msg
from ff_msgs.msg import FamCommand
from ff_hw_msgs.msg import PmcCommand
from std_msgs.msg import Float32
from std_srvs.srv import Empty

import numpy as np
from time import strftime
import os
from copy import deepcopy
import time 
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# check here how to do it properly
from test_cases import get_test_cases_list


class CtlTestManager(object):

    def __init__(self, test_, test_idx, run_name, log_path=r'/ros_logs/', description=""):
        # Test cases definition
        self.test = test_
        self.test_idx = test_idx
        
        self.current_test = 0
        self.steps_before_test = 1000
        
        # Internal properties & state & data
        self.rate_contr_hz = 62.5
        self.f_initial_state_acquired = False
        temp_mes = PoseStamped()
        self.initial_pos = temp_mes.pose.position
        self.initial_rot = temp_mes.pose.orientation
        self.initial_rot.w = 1
        self.last_pos = None
        self.last_vel = None
        self.last_fam = None
        self.last_pmc = None  # propulsion controller
        self.last_print = None
        self.last_target_pos = None
        self.step = 0

        # Loging
        self.log_folder = os.getenv('HOME') + log_path + run_name  # strftime("%Y%m%d_%H%M")
        self.prepare_log_folders(description)
        self.ros_bag = None

        # ROS stuff (node, communicattion, transormation)
        rospy.init_node('ctl_test_manager', anonymous=False)
        self.rate = rospy.Rate(self.rate_contr_hz) # 62.5hz
        self.br = tf2_ros.TransformBroadcaster()
        self.msg = geometry_msgs.msg.TransformStamped()
        self.msg.header.frame_id = "world"
        self.msg.child_frame_id = "target"
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        rospy.Subscriber("loc/truth/pose", PoseStamped, self._get_astrobee_state)
        rospy.Subscriber("loc/truth/twist", TwistStamped, self._get_astrobee_vel)
        rospy.Subscriber("gnc/ctl/command", FamCommand, self._get_astrobee_fam)
        rospy.Subscriber("hw/pmc/command", PmcCommand, self._get_astrobee_pmc)
        rospy.Subscriber("/model_print_controler_rosnode/print_status", Float32, self._get_print_length)
        self.pub_print = rospy.Publisher(r"/model_print", Float32, queue_size=5)
        

    def _get_astrobee_state(self, data):
        pos = data.pose.position
        rot = data.pose.orientation
        self.last_pos = data
        if not self.f_initial_state_acquired:
            self.f_initial_state_acquired = True
            # keep this as all 0
            self.initial_pos = pos
            #self.initial_rot = rot
            rospy.loginfo("Initialised CTL")

    def _get_astrobee_vel(self, data):
        self.last_vel = data
    
    def _get_astrobee_fam(self, data):
        self.last_fam = data

    def _get_astrobee_pmc(self, data):
        self.last_pmc = data

    def _get_print_length(self, data):
        self.last_print = data

    def generate_and_send_tf_target_and_print(self):
        if self.f_initial_state_acquired:
            target_pos, target_orient, print_comp = self.get_target()
            self.msg.header.stamp = rospy.Time.now()
            self.msg.transform.translation.x = self.initial_pos.x - target_pos[0]
            self.msg.transform.translation.y = self.initial_pos.y - target_pos[1]
            self.msg.transform.translation.z = self.initial_pos.z - target_pos[2]
            self.msg.transform.rotation.x = target_orient[0]
            self.msg.transform.rotation.y = target_orient[1]
            self.msg.transform.rotation.z = target_orient[2]
            self.msg.transform.rotation.w = target_orient[3]
            self.br.sendTransform(self.msg)
            if print_comp != 0:
                self.pub_print.publish(Float32(print_comp))

            self.last_target_pos = deepcopy(self.last_pos)
            self.last_target_pos.pose.position.x = self.msg.transform.translation.x
            self.last_target_pos.pose.position.y = self.msg.transform.translation.y
            self.last_target_pos.pose.position.z = self.msg.transform.translation.z
            self.last_target_pos.pose.orientation.x = self.msg.transform.rotation.x
            self.last_target_pos.pose.orientation.y = self.msg.transform.rotation.y
            self.last_target_pos.pose.orientation.z = self.msg.transform.rotation.z
            self.last_target_pos.pose.orientation.w = self.msg.transform.rotation.w


    def spin(self):
        test_valid = True
        while not self.test.done:
            try:
               self.time_step()
            except rospy.ROSTimeMovedBackwardsException:
                test_valid = False
                self.test.done = True
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")
                break
        self.end_test()
        return test_valid

    def time_step(self):
        self.generate_and_send_tf_target_and_print()
        self.log_test_case()
        self.step += 1
        self.rate.sleep()
    
    def end_test(self):
        self.close_ros_bag()
        self.pub_print.publish(Float32(-1.0))
        self.reset_world()


    def get_target(self):
        if self.step < self.steps_before_test:
            type_ = "default"
            pos, orient_quat = self.get_default_pose()
            print_comp = 0
            step = self.step
        elif not self.test.done:
            if self.ros_bag is None:
                self.init_new_ros_bag(self.test.get_test_name())
            pos, orient_quat, print_comp = self.test.get_target_state()
            type_ = "test {}".format(self.test_idx)
            step = self.test.step
        
        if self.step % 100 == 0:
            rospy.loginfo("running {}, step: {}, orient: {:.1f} {:.1f} {:.1f} {:.1f} \t pos: {:.2f} {:.2f} {:.2f}".format(
                    type_, step,
                    orient_quat[0], orient_quat[1], orient_quat[2], orient_quat[3],
                    pos[0], pos[1], pos[2]
                ))
        return pos, orient_quat, print_comp

    def get_default_pose(self):
        """ returns pos and rot in msg format """
        pos, orient_quat = [0, 0, 0], [self.initial_rot.x, self.initial_rot.y, self.initial_rot.z, self.initial_rot.w]
        return pos, orient_quat

    def prepare_log_folders(self, description):
        # create a log folder, description.txt, 
        if not os.path.exists(self.log_folder):
            os.mkdir(self.log_folder)
            with open(self.log_folder + '/description.txt', 'a') as dsc_file:
                dsc_file.write(description)

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
            if self.last_print is not None:
                self.ros_bag.write('/model_print_controler_rosnode/print_status', self.last_print)
            
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
    
    #def __del__(self):  # This is dirty - but context manager seems difficult to be integrated
    #    if self.ros_bag is not None:
    #        self.ros_bag.close()



if __name__ == '__main__':
    description = ""
    if len(sys.argv) < 2:
        print("usage: rosurn ctl ctl_test_manager 'description' ")
    else:
        description = sys.argv[1]

    tests = get_test_cases_list()
    
    tests_run_name = strftime("%Y%m%d_%H%M")

    test_idx = 0
    while test_idx < len(tests):
        test = tests[test_idx]
        result = False
        node = CtlTestManager(test, test_idx=test_idx, run_name=tests_run_name, description=description)
        try:
            result = node.spin()
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logerr("ROS Time Backwards! Just ignore the exception!")
        except rospy.ROSInterruptException as e:
            rospy.logerr("ROS Interrupt!! {}".format(str(e)))
            break
        if result:
            test_idx += 1
        time.sleep(10)
    rospy.logwarn("TESTS DONE")