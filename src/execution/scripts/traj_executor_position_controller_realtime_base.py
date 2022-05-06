#!/usr/bin/env python3

from math import atan2
from sympy import re
import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import simpleaudio
from nav_msgs.msg import Path
from common_functions import publish_traj_as_path, check_ctrl_c, get_executor_id
from std_msgs.msg import Bool
from drone_path_planning.msg import planning_state

from execution.msg import TrajectoryPolynomialPieceMarios

from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import tf
import uav_trajectory
import sys

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
exec_pkg_path = rospack.get_path('execution')


class TrajectoryExecutor_Position_Controller_Realtime_Base:
    def __init__(self, cf_id: int, waypoints_freq: float) -> None:
        self.cf_id = cf_id
        self.wps_freq = waypoints_freq
        self.dt = 1/self.wps_freq

        # Initialize to None to indicate that they haven't been received yet
        self.t: float = None
        self.tr: uav_trajectory.Trajectory = None

        self.odom: PoseStamped = None

        self.stabilized: bool = False

        self.load_publishers()

    def load_publishers(self):
        self.safety_land_publisher = rospy.Publisher('safety_land', String, queue_size=10)
        self.pos_pub = rospy.Publisher('reference', PoseStamped, queue_size=10)
        self.stabilized_pos_pub = rospy.Publisher('stabilized', PoseStamped, queue_size=10)

    def odometry_callback(self, odom: Odometry):
        self.odom = odom

    def wait_for_odometry(self):
        print("Waiting for odom...")
        while self.odom == None:
            check_ctrl_c()

    def wait_to_receive_traj_msg(self):
        print("LEADER :Waiting for traj matrix...")
        while type(self.tr) == type(None):
            check_ctrl_c()
            rospy.sleep(0.1)

    # threshold propably needs tuning
    def wait_until_get_to_pose(self, x, y, z, yaw, threshold=0.4, timeout_threshold=np.inf):
        # Wait until the crazyflie gets to the pose-->nomrm(error) is smaller than threshold value
        # if self.odom is None:
        #     raise Exception("No odometry received yet")

        des_pose = PoseStamped()
        des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z = x, y, z

        error = np.inf
        t0 = rospy.get_time()
        while error > threshold:
            if rospy.get_time()-t0 > timeout_threshold:
                print("Timeout occured")
                break

            des = np.array(
                [des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z])
            actual = np.array([self.odom.pose.pose.position.x,
                              self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
            # print("Waiting to go to pose...")
            error = np.linalg.norm(des - actual)
            time.sleep(0.1)

    def receive_trajectory(self, piece_pol):
        self.tr = piece_pol_to_traj(piece_pol)
        self.t = 0  # reset time in order to start from the beginning of the trajectory

    def take_off(self, height=1):
        if self.odom == None:
            rospy.logerr("No leader odometry received yet")
            sys.exit()

        x, y, z = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z

        self.go_to_pose(x, y, height, 0)

        self.wait_until_get_to_pose(
            x, y, height, 0, threshold=0.05, timeout_threshold=4)

    def go_to_pose(self, x, y, z, yaw, offset=[0, 0, 0]):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()

        x, y, z = x+offset[0], y+offset[1], z+offset[2]  # adding offset
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q[
            0], q[1], q[2], q[3]

        self.pos_pub.publish(p)

    def land(self):
        print("Landing...")
        # Land string is not necessary, but it is nice to have
        self.safety_land_publisher.publish("Land")

    def wait_to_stabilize(self, vel_thershold=0.02):
        # wait until the drone is stabilized (velocity magnitude below threshold)
        stabilized = False
        altitude = 0
        rate = rospy.Rate(20)

        while (not stabilized) or (altitude < 0.3):
            check_ctrl_c()
            # get velocities
            velocities = [self.odom.twist.twist.linear.x,
                          self.odom.twist.twist.linear.y]

            altitude = self.odom.pose.pose.position.z

            stabilized = all(abs(vel) < vel_thershold for vel in velocities)
            rate.sleep()

        self.leader_stabilized = True

    def update_time(self):
        if self.t == None:
            raise Exception("No trajectory received yet because t=None!")

        self.t += self.dt
        if self.t >= self.tr.duration:
            self.t = self.tr.duration

    def tick(self):
        if self.tr == None:
            print("No trajectory received yet...")
            return

        self.update_time()

        pos = np.array(self.tr.eval(self.t).pos)

        # create pose to publish
        pose = PoseStamped()
        pose.pose.position = Point(pos[0], pos[1], pos[2])
        pose.pose.orientation = Quaternion(0, 0, 0, 1)

        # publish pose
        self.pos_pub.publish(pose)

    def go_to_traj_start_pos(self, offset=[0, 0, 0]):
        # Go to start position
        if self.tr == None:
            rospy.logerr("No trajectory received yet")
            sys.exit()

        pos = self.tr.eval(0.0).pos
        print("Follower going to pose: {} with offset: {}".format(pos, offset))

        self.go_to_pose(pos[0], pos[1], pos[2], yaw=0, offset=offset)

    def wait_controller_to_connect(self):
        print("Waiting to connect to reference topic..")
        while self.pos_pub.get_num_connections() < 1:
            if rospy.is_shutdown():
                sys.exit()


def piece_pol_to_traj(piece_pol: TrajectoryPolynomialPieceMarios) -> uav_trajectory.Trajectory:
    t0 = rospy.Time.now()
    cfid = piece_pol.cf_id

    lines = int(len(piece_pol.poly_x)/8)

    x = np.array(piece_pol.poly_x).reshape((lines, 8))
    y = np.array(piece_pol.poly_y).reshape((lines, 8))
    z = np.array(piece_pol.poly_z).reshape((lines, 8))
    yaw = np.zeros((lines, 8))
    durations = np.array(piece_pol.durations).reshape((lines, 1))

    matrix = np.zeros((lines, 1+8*4))
    matrix[:, 0] = durations.flatten()
    matrix[:, 1:9] = x
    matrix[:, 9:17] = y
    matrix[:, 17:25] = z
    matrix[:, 25:33] = yaw

    tr = uav_trajectory.Trajectory()
    tr.load_from_matrix(matrix)

    return tr
