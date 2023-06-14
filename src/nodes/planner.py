#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal
import math

import numpy as np
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json


class BasePlanner:
    def __init__(self):
        pass
    def get_commands(self, goals_pos=None, obstacles_pos=None):
        raise NotImplementedError()


class XYPotentialPlanner(BasePlanner):
    def __init__(self):
        self.k_att = np.array([1., 1.])
        self.k_rep = np.array([0.5, 0.5])

    def get_attraction_forces(self, att_pos_list: np.ndarray):
        if att_pos_list is None:
            return np.zeros_like(self.k_att)
        else:
            assert len(att_pos_list.shape) == 2
            att_force = att_pos_list.sum(1) * self.k_att
            return att_force

    def get_repulsion_forces(self, rep_pos_list: np.ndarray):
        if rep_pos_list is None:
            return np.zeros_like(self.k_rep)
        else:
            assert len(rep_pos_list.shape) == 2
            rep_force = rep_pos_list.sum(1) * self.k_rep
            return rep_force

    def compute_commands(self, goals_pos, obstacles_pos):
        force = self.get_attraction_forces(goals_pos) + self.get_repulsion_forces(obstacles_pos)
        return force


class Planner:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('Planner', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.force_coef = 0.15
        self.torq_coef = 0.1

        # Initialize subscriber
        self.map_sub = rospy.Subscriber('/map', String, self.map_callback)
        self.map = None
        # TODO: another subscriber
        # self.sensor_sub = rospy.Subscriber('', TYPE, self.sensor_callback)
        # Intialize publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = None
        self.rate = rospy.Rate(10)  # Publisher frequency
        # Motion planner
        self.motion_planner = XYPotentialPlanner()


    def map_callback(self, msg):
        self.map = json.loads(msg.data)
        self.cmd = geometry_msgs.msg.Twist()

        # XY-translation
        dist_goal = np.array([[self.map['/goal'][0], self.map['/goal'][1]]])
        xy_commands = self.motion_planner.get_commands(goals_pos=dist_goal, obstacles_pos=None)
        self.cmd.linear.x = self.force_coef * xy_commands[0]
        self.cmd.linear.y = self.force_coef * xy_commands[1]

        # Z-rotation
        self.cmd.angular.z = self.torq_coef * math.atan2(dist_goal[0], dist_goal[1])

    # sensor callback
    # if distance > threhold:
    # self.dist_obs_x = 0
    # else:
    # self.dist_obs_x = read from sensor

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
