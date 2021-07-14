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


def get_test_cases_list():
    tests = [
            #ModelIdentificationTest(p_x=-0.4),
            #ModelIdentificationTest(r_z=45),
            #StationaryPrintEnd(),
            #QuickTest(),
            #SimpleRotationTest(value=2, print_length_cm=0),
            #SimpleRotationTest(value=35, print_length_cm=0),
            #SimpleRotationTest(value=35, print_length_cm=60),
            #SimpleRotationTest(value=35, print_length_cm=120),
            SimpleRotationTest(value=35, print_length_cm=180),
            #SimpleRotationTest(value=35, print_length_cm=240),
            #SimpleRotationTest(value=35, print_length_cm=300),
            #SimpleTranslationTest(value=1, print_length_cm=240),
            #SimpleTranslationTest(value=0.4, print_length_cm=0),
            #TranslationWithRotationTest(print_length_cm=240),
##
            #SimpleRotationTest(value=35, print_length_cm=0, printing_speed=0.03/1000),
            #
            #SimpleRotationTest(print_length_cm=60),
            #SimpleTranslationTest(print_length_cm=60),
            #TranslationWithRotationTest(print_length_cm=60),
            #StationaryPrintEndWithRotation()
            ]
    
    return tests

# This should be     
class TestCaseClass(object):
    def __init__(self):
        self.step = 0
        self.done = False
        self.test_case_name = None   # This needs to be set in the child class -> TODO: in future create a abstract set_method 

    def get_target_state(self):
        current_target = [r for r in self.actions_sequence if (self.step in r)][0]
        #current_target = current_target[0]
        pos = current_target.get_pos(self.step)
        orientation_quat =  current_target.get_quat(self.step)
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
        self.test_case_name = "2_Simple_rotation_{}deg_print_step_{}um_print_from_{}cm".format(value, int(printing_speed*10**6), print_length_cm)
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  999, (0, 0, 0), (0, 0 , 0)),
            Action(1000, 2999, (0, 0, 0), (value, 0 , 0), printing_speed),
            Action(3000, 4999, (0, 0, 0), (0, 0 , 0), printing_speed),
            Action(5000, 6999, (0, 0, 0), (0, value , 0), printing_speed),
            Action(7000, 8999, (0, 0, 0), (0, 0 , 0), printing_speed),
            Action(9000, 10999, (0, 0, 0), (0, 0 , value), printing_speed),
            Action(11000, 12999, (0, 0, 0), (0, 0 , 0), printing_speed),
        ]
        self.length = self.actions_sequence[-1].t_end

class SimpleTranslationTest(TestCaseClass):
    def __init__(self, value=0.4, printing_speed=0, print_length_cm=0):
        super(SimpleTranslationTest, self).__init__()
        self.test_case_name = "3_Simple_translation_{}cm_print_step_{}um_print_from_{}cm".format(int(value*100), int(printing_speed*10**6), print_length_cm)
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  999, (0, 0, 0), (0, 0 , 0)),
            Action(1000, 2999, (0, value, 0), (0, 0 , 0), printing_speed),
            Action(3000, 4999, (0, value, value), (0, 0 , 0), printing_speed),
            Action(5000, 6999, (value, value, value), (0, 0 , 0), printing_speed),
            Action(7000, 8999, (value, 0, value), (0, 0 , 0), printing_speed),
            Action(9000, 10999, (value, 0, 0), (0, 0 , 0), printing_speed),
            Action(11000, 12999, (0, 0, 0), (0, 0 , 0), printing_speed),
        ]
        self.length = self.actions_sequence[-1].t_end

class TranslationWithRotationTest(TestCaseClass):
    def __init__(self, translation_value=0.4, rotation_value=30, printing_speed=0, print_length_cm=0):
        super(TranslationWithRotationTest, self).__init__()
        self.test_case_name = "4_Rotation_with_translation_{}deg_{}cm_print_step_{}um_print_from_{}cm".format(rotation_value, int(translation_value*100), int(printing_speed*10**6), print_length_cm)
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  999, (0, 0, 0),       (0, 0 , 0)),
            Action(1000, 2999, (0, 0, translation_value),     (0, 0 , rotation_value), printing_speed),
            Action(3000, 4999, (0, 0, -translation_value),     (0, 0 , 2*rotation_value), printing_speed),
            Action(5000, 6999, (0, 0, 0),     (0, 0 , 0), printing_speed),
        ]
        self.length = self.actions_sequence[-1].t_end

class StationaryPrintEnd(TestCaseClass):
    def __init__(self, printing_speed=0.1/1000, print_length_cm=0):
        super(StationaryPrintEnd, self).__init__()
        self.test_case_name = "5_Stationary_print_end_print_step_{}um_print_from_{}cm".format(int(printing_speed*10**6), print_length_cm)
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  999, (0, 0, 0),       (0, 0 , 0)),
            ActionStationaryPrintEnd(1000, 9999, (0, 0, 0), (0, 0 , 0), printing_speed),
        ]
        self.length = self.actions_sequence[-1].t_end

class StationaryPrintEndWithRotation(TestCaseClass):
    def __init__(self, printing_speed=0.03/1000, print_length_cm=0):
        super(StationaryPrintEndWithRotation, self).__init__()
        self.test_case_name = "6_Stationary_print_end_with_rotation_print_step_{}um_print_from_{}cm".format(int(printing_speed*10**6), print_length_cm)
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  999, (0, 0, 0),       (0, 0 , 0)),
            ActionStationaryPrintEndWithRotation(1000, 12999, (0, 0, 0), (0, 0 , 0), printing_speed),
        ]
        self.length = self.actions_sequence[-1].t_end
        

class QuickTest(TestCaseClass):
    def __init__(self):
        super(QuickTest, self).__init__()
        self.test_case_name = "0_QuickTest"
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), 0),
            Action(   1,  999, (1, 1, 1), (0, 0 , 0), 0),
        ]
        self.length = self.actions_sequence[-1].t_end


class ModelIdentificationTest(TestCaseClass):
    def __init__(self, p_x=0, p_y=0, p_z=0, r_x=0, r_y=0, r_z=0, print_length_cm=0):
        super(ModelIdentificationTest, self).__init__()
        self.test_case_name = "1_ModelIdentificationTest"
        self.actions_sequence = [
            Action(   0,  0, (0, 0, 0), (0, 0 , 0), float(print_length_cm)/100.0),
            Action(   1,  99, (0, 0, 0), (0, 0 , 0), 0),
            Action(   100,  2999, (p_x, p_y, p_z), (r_x, r_y , r_z), 0),
        ]
        self.length = self.actions_sequence[-1].t_end

    
class Action(object):
    def __init__(self, t_beg, t_end, pos, rot, print_speed=0):
        self.t_0 = t_beg
        self.t_end = t_end
        self.pos = pos
        self.rot = rot
        self.print_speed = print_speed
    def __contains__(self, t_):
        return self.t_0 <= t_ and self.t_end >= t_
    def get_quat(self, *args):
        """return list [x, y, z, w]"""
        rot_rad = [np.deg2rad(a_deg) for a_deg in self.rot]  # How about np.deg2rad(np.clip(a_deg, -89, 89)) ?
        return quaternion_from_euler(*rot_rad)
    def get_pos(self, *args):
        return self.pos
    def get_pos_vel(self):
        return [0, 0, 0]
    def get_ort_vel(self):
        return [0, 0, 0]

class ActionStationaryPrintEnd(Action):
    def __init__(self, t_beg, t_end, pos, rot, print_speed):
        super(ActionStationaryPrintEnd, self).__init__(t_beg, t_end, pos, rot, print_speed)
        self.start_time = None
    def get_pos(self, time):
        if self.start_time is None:
            self.start_time = time
        time -= self.start_time
        current_length = time * self.print_speed
        pos = list(self.pos)
        pos[0] += current_length
        return pos
    def get_pos_vel(self):
        return [self.print_speed, 0, 0] 
    
class ActionStationaryPrintEndWithRotation(Action):
    def __init__(self, t_beg, t_end, pos, rot, print_speed):
        super(ActionStationaryPrintEndWithRotation, self).__init__(t_beg, t_end, pos, rot, print_speed)
        self.total_time = t_end - t_beg
        self.start_time = None
        self.angle_change = 90  # deg
        self.printing_part = 0.95
        self.l0 = 0.1

    def get_pos(self, time):
        if self.start_time is None:
            self.start_time = time
        time -= self.start_time
        time = min(time, self.printing_part * self.total_time)

        current_length = time * self.print_speed + self.l0
        current_angle = self.angle_change * time / (self.total_time * self.printing_part)
        current_x = current_length * np.cos(np.deg2rad(current_angle))
        current_y = current_length * np.sin(np.deg2rad(current_angle))
        pos = list(self.pos)
        pos[0] += current_x - self.l0
        pos[1] += current_y
        return pos

    def get_quat(self, time, *args):
        """return list [x, y, z, w]"""
        if self.start_time is None:
            self.start_time = time
        time -= self.start_time
        current_angle = self.angle_change * time / self.total_time
        rot_rad = [np.deg2rad(a_deg) for a_deg in self.rot]  # How about np.deg2rad(np.clip(a_deg, -89, 89)) ?
        rot_rad[2] += np.deg2rad(current_angle)
        return quaternion_from_euler(*rot_rad)
    
    def get_pos_vel(self, time):
        if self.start_time is None:
            self.start_time = time
        time -= self.start_time
        if time > self.printing_part * self.total_time:
            vel_x, vel_y, vel_z =  0, 0, 0
        else:
            angular_vel = np.rad2deg(self.angle_change / (self.total_time * self.printing_part))
            vel_x = self.print_speed * np.cos(angular_vel * time) - self.print_speed * time * angular_vel * np.sin(angular_vel * time)
            vel_y = self.print_speed * np.sin(angular_vel * time) - self.print_speed * time * angular_vel * np.cos(angular_vel * time)
            vel_z = 0
        
        return [vel_x, vel_y, vel_z]