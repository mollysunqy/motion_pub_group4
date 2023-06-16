#!/usr/bin/env python3
# Reads the map output (see map_broadcaster.py) and publishes twist commands to reach the goal
import math

import numpy as np
import rospy
from std_msgs.msg import String
import geometry_msgs.msg
import json


class XYBasePlanner:
    def compute_commands(self, goals_pos: np.ndarray, obstacles_pos: np.ndarray) -> np.ndarray:
        """
        Takes goal and obstacle positions as input and computes the velocities to apply to the robot
        :param goals_pos / obstacles_pos: ndarray of shape (n, 2) containing relative positions of goals / obstacles
        :return: ndarray of shape (2,) containing XY velocities to apply to the robot
        """
        raise NotImplementedError()


class XYPotentialBasedPlanner(XYBasePlanner):
    def __init__(self):
        self.k_att = np.array([1., 1.])  # HARDCODED
        self.k_rep = np.array([0.2, 0.2])  # HARDCODED
        self.object_radius = 0.2  # HARDCODED
        self.rep_dist_threshold = 1.  # HARDCODED

    def get_attraction_forces(self, att_pos_list: np.ndarray) -> np.ndarray:
        if len(att_pos_list) == 0:
            return np.zeros_like(self.k_att)
        else:
            assert len(att_pos_list.shape) == 2
            assert att_pos_list.shape[1] == 2
            att_force = att_pos_list.sum(0) * self.k_att
            return att_force #+ np.array([0., 0.2])

    def get_repulsion_forces(self, rep_pos_list: np.ndarray) -> np.ndarray:
        if len(rep_pos_list) == 0:
            return np.zeros_like(self.k_rep)
        else:
            assert len(rep_pos_list.shape) == 2
            assert rep_pos_list.shape[1] == 2
            dist_to_surface = max(np.linalg.norm(rep_pos_list, axis=1) - self.object_radius, 0.)
            rep_force_mask = np.resize((dist_to_surface < self.rep_dist_threshold).astype(np.float), rep_pos_list.shape)
            rep_force = ((1. / (rep_pos_list - self.object_radius)) * rep_force_mask).sum(0) * self.k_rep
            return rep_force

    def compute_commands(self, goals_pos: np.ndarray, obstacles_pos: np.ndarray) -> np.ndarray:
        force = self.get_attraction_forces(goals_pos) - self.get_repulsion_forces(obstacles_pos)
        return force


class Planner:
    def __init__(self):
        super().__init__()

        # Initialize Node
        rospy.init_node('Planner', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.force_coef = 0.2  # HARDCODED
        self.torq_coef = 0.4  # HARDCODED

        # Initialize subscriber
        self.map_sub = rospy.Subscriber('/map', String, self.map_callback)
        self.map = None

        # Intialize publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.cmd = None
        self.rate = rospy.Rate(10)  # Publisher frequency

        # Motion planner
        self.motion_planner = XYPotentialBasedPlanner()
        self.look_direction = "goal"  # "forward" or "goal"
        self.zRotation_angle_threshold = 0.3  # HARDCODED

    def map_callback(self, msg):
        self.map = json.loads(msg.data)
        self.cmd = geometry_msgs.msg.Twist()

        # XY-translation
        dist_goal = np.array([[self.map['/goal'][0], self.map['/goal'][1]]])
        dist_obstacles = []
        for key in self.map.keys():
            if 'obstacle' in key:
                dist_obstacles.append([self.map[key][0], self.map[key][1]])

        xy_commands = self.motion_planner.compute_commands(goals_pos=dist_goal, obstacles_pos=np.array(dist_obstacles))
        x_command = self.force_coef * xy_commands[0]
        y_command = self.force_coef * xy_commands[1]

        # Z-rotation
        if self.look_direction == "goal":
            goal_angle = math.atan2(dist_goal[0][1], dist_goal[0][0])
            z_command = self.torq_coef * goal_angle
        elif self.look_direction == "forward":
            commands_angle = math.atan2(xy_commands[1], xy_commands[0])
            if np.abs(commands_angle) > self.zRotation_angle_threshold:
                z_command = self.torq_coef * commands_angle * np.linalg.norm(np.array([x_command, y_command]))
            else:
                z_command = self.torq_coef * commands_angle
        else:
            raise NotImplementedError(f"Invalid self.look_direction={self.look_direction}")

        # clipping
        x_command = min(max(x_command, -0.4), 0.4)
        y_command = min(max(y_command, -0.4), 0.4)
        z_command = min(max(z_command, -1.), 1.)

        # send commands
        self.cmd.linear.x = x_command
        self.cmd.linear.y = y_command
        self.cmd.angular.z = z_command

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