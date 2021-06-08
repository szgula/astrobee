#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped

from tf.transformations import quaternion_from_euler, euler_from_quaternion
import geometry_msgs.msg

# THIS IS NOT NEEDED AS:
# rosrun tf tf_echo world truth
# do the job
class AstrobeeTf2Publisher:
    def __init__(self):
        rospy.init_node('astrobee_tf2_publisher', anonymous=False)
        rospy.Subscriber("loc/truth/pose", PoseStamped, self._get_astrobee_state)
        self.br = tf2_ros.TransformBroadcaster()
        self.msg = geometry_msgs.msg.TransformStamped()
        rospy.spin()

    def _get_astrobee_state(self, data):
        pos = data.pose.position
        rot = data.pose.orientation

        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "world"
        self.msg.child_frame_id = "body"
        self.msg.transform.translation.x = pos.x
        self.msg.transform.translation.y = pos.y
        self.msg.transform.translation.z = pos.z
        self.msg.transform.rotation.x = rot.x
        self.msg.transform.rotation.y = rot.y
        self.msg.transform.rotation.z = rot.z
        self.msg.transform.rotation.w = rot.w
        self.br.sendTransform(self.msg)
        #print(self.msg)


if __name__ == '__main__':
    try:
        node = AstrobeeTf2Publisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass