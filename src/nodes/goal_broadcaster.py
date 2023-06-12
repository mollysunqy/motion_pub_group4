#!/usr/bin/env python3
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('goal_broadcaster')
    br = tf.TransformBroadcaster()

    # Parameter to control object (works only with boards!!!)
    rospy.set_param("/mrss_motion/goal_object", "board2")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        goal_object = rospy.get_param("/mrss_motion/goal_object")

        # .8 m in front of object
        br.sendTransform((0.0, 0.0, 0.6),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "goal",
                         goal_object)
        rate.sleep()
