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

class CtlTestManager(object):

    def __init__(self, log_path=r'/ros_logs/'):
        # Test cases definition
        self.tests = [
            SimpleRotationTest(print_length_cm=0),
            SimpleTranslationTest(print_length_cm=0),
            TranslationWithRotationTest(print_length_cm=0),

            RotationWhilePriningTest(print_length_cm=0),
            
            SimpleRotationTest(print_length_cm=60),
            SimpleTranslationTest(print_length_cm=60),
            TranslationWithRotationTest(print_length_cm=60)]
        
        self.current_test = 0
        self.time_since_last_test_ended = 0
        self.time_between_tests = 1000
        
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
        while True:
            try:
               self.time_step()
            except rospy.ROSTimeMovedBackwardsException:
                rospy.logerr("ROS Time Backwards! Just ignore the exception!")

    def time_step(self):
        self.generate_and_send_tf_target_and_print()
        self.log_test_case()
        self.step += 1
        self.rate.sleep()

    def get_target(self):
        current_test = self.current_test
        if (current_test < len(self.tests)) and (not self.tests[current_test].done):
            if self.ros_bag is None:
                self.init_new_ros_bag(self.tests[current_test].get_test_name())
            pos, orient_quat, print_comp = self.tests[current_test].get_target_state()
            type_ = "test {}".format(current_test)
            step = self.tests[current_test].step
        else:
            type_ = "default"
            self.close_ros_bag()
            pos, orient_quat = self.get_default_pose()
            print_comp = 0
            if self.time_since_last_test_ended == 0:
                # This should reset the siumlation
                self.reset_world()
                self.pub_print.publish(Float32(-1.0))
            step = self.time_since_last_test_ended
            self.time_since_last_test_ended +=1
            if (self.time_since_last_test_ended > self.time_between_tests) and (current_test < len(self.tests) - 1):
                self.time_since_last_test_ended = 0
                self.current_test += 1
        
        if self.step % 100 == 0:
            rospy.loginfo("running {}, step: {}, time {}, {}, \t orient: {:.1f} {:.1f} {:.1f} {:.1f} \t pos: {:.2f} {:.2f} {:.2f}".format(
                    type_, step, rospy.Time.now().to_sec(), rospy.get_rostime(),
                    orient_quat[0], orient_quat[1], orient_quat[2], orient_quat[3],
                    pos[0], pos[1], pos[2]
                ))
        return pos, orient_quat, print_comp

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
        return pos, orientation_quat, current_target.print_speed

    def get_test_name(self):
        return self.test_case_name
    
    
class SimpleRotationTest(TestCaseClass):
    def __init__(self, value=66, printing_speed=0, print_length_cm=0):
        super(SimpleRotationTest, self).__init__()
        self.rotation_value = value
        self.test_case_name = "Simple_rotation_{}deg_print_{}cm".format(value, print_length_cm)
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  999, (0, 0, 0), (0, 0 , 0)),
            Action(1000, 1999, (0, 0, 0), (value, 0 , 0), printing_speed),
            Action(2000, 2999, (0, 0, 0), (0, 0 , 0), printing_speed),
            Action(3000, 3999, (0, 0, 0), (0, value , 0), printing_speed),
            Action(4000, 4999, (0, 0, 0), (0, 0 , 0), printing_speed),
            Action(5000, 5999, (0, 0, 0), (0, 0 , value), printing_speed),
            Action(6000, 6999, (0, 0, 0), (0, 0 , 0), printing_speed),
        ]
        self.length = 6999

class SimpleTranslationTest(TestCaseClass):
    def __init__(self, value=0.4, printing_speed=0, print_length_cm=0):
        super(SimpleTranslationTest, self).__init__()
        self.test_case_name = "Simple_translation_{}cm_print_{}cm".format(int(value*100), print_length_cm)
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  999, (0, 0, 0), (0, 0 , 0)),
            Action(1000, 1999, (0, value, 0), (0, 0 , 0), printing_speed),
            Action(2000, 2999, (0, value, value), (0, 0 , 0), printing_speed),
            Action(3000, 3999, (value, value, value), (0, 0 , 0), printing_speed),
            Action(4000, 4999, (value, 0, value), (0, 0 , 0), printing_speed),
            Action(5000, 5999, (value, 0, 0), (0, 0 , 0), printing_speed),
            Action(6000, 6999, (0, 0, 0), (0, 0 , 0), printing_speed),
        ]
        self.length = 6999

class TranslationWithRotationTest(TestCaseClass):
    def __init__(self, translation_value=0.4, rotation_value=50, printing_speed=0, print_length_cm=0):
        super(TranslationWithRotationTest, self).__init__()
        self.test_case_name = "Translation_with_rotation_print_{}cm".format(print_length_cm)
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  999, (0, 0, 0),       (0, 0 , 0)),
            Action(1000, 1999, (0, 0, 0),       (0, 0 , rotation_value), printing_speed),
            Action(2000, 2999, (0, translation_value, 0),     (0, 0 , rotation_value), printing_speed),
            Action(3000, 3999, (0, translation_value, 0),     (rotation_value, 0 , rotation_value), printing_speed),
            Action(4000, 4999, (translation_value, translation_value, 0),   (rotation_value, 0 , rotation_value), printing_speed),
            Action(5000, 5999, (translation_value, translation_value, translation_value), (0, 0 , rotation_value), printing_speed),
            Action(6000, 6999, (0, 0, 0),       (0, 0 , 0), printing_speed),
        ]
        self.length = 6999


class RotationWhilePriningTest(TestCaseClass):
    def __init__(self, value=66, printing_speed=0.1/1000, print_length_cm=0):
        super(RotationWhilePriningTest, self).__init__()
        self.rotation_value = value
        self.test_case_name = "Rotation_{}deg_print_step_{}um_print_from_{}".format(value, int(printing_speed*10**6), print_length_cm)
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  999, (0, 0, 0), (0, 0 , 0)),
            Action(1000, 1999, (0, 0, 0), (value, 0 , 0), printing_speed),
            Action(2000, 2999, (0, 0, 0), (0, 0 , 0), printing_speed),
            Action(3000, 3999, (0, 0, 0), (0, value , 0), printing_speed),
            Action(4000, 4999, (0, 0, 0), (0, 0 , 0), printing_speed),
            Action(5000, 5999, (0, 0, 0), (0, 0 , value), printing_speed),
            Action(6000, 6999, (0, 0, 0), (0, 0 , 0), printing_speed),
        ]
        self.length = 6999
        

    
class Action(object):
    def __init__(self, t_beg, t_end, pos, rot, print_speed=0):
        self.t_0 = t_beg
        self.t_end = t_end
        self.pos = pos
        self.rot = rot
        self.print_speed = print_speed
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
    except rospy.ROSTimeMovedBackwardsException:
        rospy.logerr("ROS Time Backwards! Just ignore the exception!")
    except rospy.ROSInterruptException as e:
        rospy.logerr("ROS Interrupt!! {}".format(str(e)))
    except Exception as e:
        rospy.logerr("ROS Time Backwards! Just ignore the exception! {}".format(str(e)))
        print(str(e))