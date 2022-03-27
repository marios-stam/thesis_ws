#!/usr/bin/env python3
# print working directory
import sys
from drone_path_planning.msg import rigid_body_dynamic_path
from nav_msgs.msg import Odometry
import rospy
from math import atan2, pi
from RigidBodyPlanners import *

from drone_path_planning.msg import planning_state
from common_functions import get_leader_follower_names


class PathPlanningStart:
    def __init__(self):
        self.leader_pos = None
        self.follower_pos = None
        self.z_offset = 0.5
        self.published_once = False

    def pos_callback_leader(self, msg):
        self.leader_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.check_if_ready()

    def pos_callback_follower(self, msg):
        self.follower_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.check_if_ready()

    def check_if_ready(self):
        if not self.published_once and self.leader_pos is not None and self.follower_pos is not None:
            l_pos = self.leader_pos
            f_pos = self.follower_pos
            # get rigid body start coords
            rb_x = min(l_pos[0], f_pos[0]) + abs(l_pos[0]-f_pos[0])/2
            rb_y = min(l_pos[1], f_pos[1]) + abs(l_pos[1]-f_pos[1])/2
            rb_z = min(l_pos[2], f_pos[2]) + abs(l_pos[2]-f_pos[2])/2
            rb_z += self.z_offset

            rb_pos = [rb_x, rb_y, rb_z]
            rb_yaw = 0
            drones_distance = abs(rb_x)
            theta = atan2(l_pos[2]-f_pos[2], l_pos[0]-f_pos[0])
            rb_state = [rb_x, rb_y, rb_z, rb_yaw, drones_distance, theta]
            print("Rigid body start state:", rb_state)
            state = planning_state()
            state.x, state.y, state.z, state.yaw, state.drones_distance, state.drones_angle = rb_state
            start_planning_publisher.publish(state)
            print("Published planning start state")

            self.published_once = True


if __name__ == "__main__":
    rospy.init_node("path_planning_start")

    leader, follower = get_leader_follower_names()
    print("Leader:", leader)
    print("Follower:", follower)
    print('/pixy/vicon/{}/{}/odom'.format(leader, leader))
    print('/pixy/vicon/{}/{}/odom'.format(follower, follower))
    path_planning_start = PathPlanningStart()
    rospy.Subscriber('/pixy/vicon/{}/{}/odom'.format(leader, leader), Odometry, path_planning_start.pos_callback_leader)
    rospy.Subscriber('/pixy/vicon/{}/{}/odom'.format(follower, follower), Odometry, path_planning_start.pos_callback_follower)
    start_planning_publisher = rospy.Publisher('/start_planning', planning_state, queue_size=10)

    rospy.spin()
