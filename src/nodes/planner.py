#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal
import math

import numpy as np
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json


class Planner:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('Planner', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        # Initialize subscriber
        self.map_sub = rospy.Subscriber('/map', String, self.map_callback)
        self.map = None
        self.threshold = 0.01
        # Intialize publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = None
        self.rate = rospy.Rate(10)  # Publisher frequency
        self.robot_pose_topic = "string"
        # getting the robot position
        # self.map_sub = rospy.Subscriber(self.robot_pose_topic, String, self.map_callback)

        # TODO BEGIN MRSS: Add attributes (If needed)

        # END MRSS

    def map_callback(self, msg):
        self.map = json.loads(msg.data)

        # TODO BEGIN MRSS: Use map for planning
        dist_goal_x = self.map['/goal'][0]
        dist_goal_y = self.map['/goal'][1]
        dist_goal_orient = self.map['/goal'][2]

        robot_orient = self.map['/rig'] #TODO: fill in

        # END MRSS

        # Twist
        self.cmd = geometry_msgs.msg.Twist()
        if np.linalg.norm([dist_goal_x,dist_goal_y])<self.threshold:
            # turn to goal orientation
            self.cmd.angular.z = 0.1 * dist_goal_orient
        else:
            # go to goal
            self.cmd.angular.z = 0.1  * math.atan2(dist_goal_x,dist_goal_y)
        # TODO BEGIN MRSS: Update the current command
        self.cmd.linear.x = 0.15 * dist_goal_x
        self.cmd.linear.y = 0.15 * dist_goal_y
        # END MRSS

    def spin(self):
        '''
        Spins the node.
        '''
        try:
            while not rospy.is_shutdown():
                if self.cmd is not None:
                    # Publish
                    self.cmd_pub.publish(self.cmd)
                else:
                    rospy.logwarn("SKIP")

                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down planner.")

    def on_shutdown(self):
        '''
        Called on node shutdown.
        '''
        pass


if __name__ == '__main__':
    try:
        node = Planner()
        node.spin()
    except rospy.ROSInterruptException:
        pass
