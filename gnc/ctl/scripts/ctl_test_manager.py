#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import geometry_msgs.msg
import numpy as np
print("imports done")

class CtlTestManager(object):

    def __init__(self):
        self.tests = [
            SimpleRotationTest(),
            SimpleTranslationTest()]

        print("starting...")
        rospy.init_node('ctl_test_manager', anonymous=False)
        self.br = tf2_ros.TransformBroadcaster()
        self.msg = geometry_msgs.msg.TransformStamped()
        self.rate_contr_hz = 62.5
        self.rate = rospy.Rate(self.rate_contr_hz) # 62.5hz
        self.f_initial_state_acquired = False
        temp_mes = PoseStamped()
        self.initial_pos = temp_mes.pose.position
        self.initial_rot = temp_mes.pose.orientation
        rospy.Subscriber("loc/truth/pose", PoseStamped, self._get_astrobee_state)

    def _get_astrobee_state(self, data):
        pos = data.pose.position
        rot = data.pose.orientation
        if self.f_initial_state_acquired:
            target_pos, target_orient = self.get_target()
            self.msg.header.stamp = rospy.Time.now()
            self.msg.header.frame_id = "world"
            self.msg.child_frame_id = "target"
            self.msg.transform.translation.x = self.initial_pos.x - target_pos[0]
            self.msg.transform.translation.y = self.initial_pos.y - target_pos[1]
            self.msg.transform.translation.z = self.initial_pos.z - target_pos[2]
            self.msg.transform.rotation.x = target_orient[0]
            self.msg.transform.rotation.y = target_orient[1]
            self.msg.transform.rotation.z = target_orient[2]
            self.msg.transform.rotation.w = target_orient[3]
            self.br.sendTransform(self.msg)
            #print("msg: ", self.msg.transform)
        else:
            self.f_initial_state_acquired = True
            self.initial_pos = pos
            self.initial_rot = rot
            print("Initialised")

    def spin(self):
        #while True:
        #    self.rate.sleep()
        rospy.spin()

    def get_target(self):
        if not self.tests[1].done:
            pos, orient_quat = self.tests[1].get_target_state()
            print('running test', " step: {} orientation: {:.1f} {:.1f} {:.1f} {:.1f} \t position: {:.2f} {:.2f} {:.2f}".format(
                self.tests[1].step,
                orient_quat[0], orient_quat[1], orient_quat[2], orient_quat[3],
                pos[0], pos[1], pos[2]
            ))
        else:
            print('running default')
            pos, orient_quat = self.get_default_pose()
        return pos, orient_quat

    def get_default_pose(self):
        """ returns pos and rot in msg format """
        pos, orient_quat = [self.initial_pos.x, self.initial_pos.y, self.initial_pos.z], [self.initial_rot.x, self.initial_rot.y, self.initial_rot.z, self.initial_rot.w]
        return pos, orient_quat




# This should be     
class TestCaseClass(object):
    def __init__(self):
        self.step = 0
        self.done = False
    def get_target_state(self):
        current_target = [r for r in self.actions_sequence if (self.step in r)][0]
        #current_target = current_target[0]
        pos = current_target.pos
        orientation_quat =  current_target.get_quat()
        self.step += 1
        if self.step >= self.length:
            self.done = True
        return pos, orientation_quat
    
    
class SimpleRotationTest(TestCaseClass):
    def __init__(self):
        super(SimpleRotationTest, self).__init__()
        self.actions_sequence = [
            Action(   0,  999, (0, 0, 0), (0, 0 , 0)),
            Action(1000, 1999, (0, 0, 0), (70, 0 , 0)),
            Action(2000, 2999, (0, 0, 0), (0, 0 , 0)),
            Action(3000, 3999, (0, 0, 0), (0, 70 , 0)),
            Action(4000, 4999, (0, 0, 0), (0, 0 , 0)),
            Action(5000, 5999, (0, 0, 0), (0, 0 , 70)),
            Action(6000, 6999, (0, 0, 0), (0, 0 , 0)),
        ]
        self.length = 6999

class SimpleTranslationTest(TestCaseClass):
    def __init__(self):
        super(SimpleTranslationTest, self).__init__()
        self.actions_sequence = [
            Action(   0,  999, (0, 0, 0), (0, 0 , 0)),
            Action(1000, 1999, (0, 0.4, 0), (0, 0 , 0)),
            Action(2000, 2999, (0, 0.4, 0.4), (0, 0 , 0)),
            Action(3000, 3999, (0.4, 0.4, 0.4), (0, 0 , 0)),
            Action(4000, 4999, (0.4, 0, 0.4), (0, 0 , 0)),
            Action(5000, 5999, (0.4, 0, 0), (0, 0 , 0)),
            Action(6000, 6999, (0, 0, 0), (0, 0 , 0)),
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
        rot_rad = [np.deg2rad(a_deg) for a_deg in self.rot]
        return quaternion_from_euler(*rot_rad)


if __name__ == '__main__':
    print("get into main")
    try:
        node = CtlTestManager()
        node.spin()
    except rospy.ROSInterruptException:
        pass