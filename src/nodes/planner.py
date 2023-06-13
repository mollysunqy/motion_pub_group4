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
        # Intialize publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = None
        self.rate = rospy.Rate(10)  # Publisher frequency
        # Hyperparams
        self.y_dir_preference = 0.02


    def map_callback(self, msg):
        self.map = json.loads(msg.data)

        dist_goal_x = self.map['/goal'][0]
        dist_goal_y = self.map['/goal'][1]
        dist_goal_orient = self.map['/goal'][2]

        dist_obstacle_x = self.map['/obstacle'][0]  # TODO: make sure the key is right
        dist_obstacle_y = self.map['/obstacle'][1]

        # Z-rotation
        self.cmd = geometry_msgs.msg.Twist()

        # XY-translation
        self.cmd.linear.x = 0.15 ( dist_goal_x - 2 * dist_obstacle_x )
        self.cmd.linear.y = 0.15 ( dist_goal_y - 2 * dist_obstacle_y + self.y_dir_preference )
        self.cmd.angular.z = 0.1  * math.atan2(dist_goal_x,dist_goal_y)


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
