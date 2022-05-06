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
from traj_executor_position_controller_realtime_base import TrajectoryExecutor_Position_Controller_Realtime_Base

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
exec_pkg_path = rospack.get_path('execution')


class TrajectoryExecutor_Position_Controller_Leader(TrajectoryExecutor_Position_Controller_Realtime_Base):
    def __init__(self, cf_id: int, waypoints_freq: float) -> None:
        super().__init__(cf_id, waypoints_freq)

        self.follower_stabilized_pos = None
        self.follower_stabilized = False

    def follower_land(self):
        print("Follower landing...")
        # Land string is not necessary, but it is nice to have
        follower_safety_land_publisher.publish("Land")

    def wait_follower_to_stabilize(self):
        rate = rospy.Rate(20)
        while self.follower_stabilized_pos == None:
            check_ctrl_c()
            rate.sleep()

    def follower_stabilized_callback(self, msg: PoseStamped):
        # Follower is stabilized and then sends its position to the leader
        self.follower_stabilized_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        print("Follower stabilized at position:", self.follower_stabilized_pos)
        self.follower_stabilized = True

    def publish_sync_signal(self):
        sync_pub.publish(True)

    def land_all(self):
        self.land()
        self.follower_land()


class TrajectoryExecutor_Position_Controller:
    def __init__(self, cf_id: int, waypoints_freq: float) -> None:
        self.cf_id = cf_id
        self.wps_freq = waypoints_freq
        self.dt = 1/self.wps_freq

        # Initialize to None to indicate that they haven't been received yet
        self.t = None

        self.odom = None
        self.follower_stabilized_pos = None

        self.leader_stabilized = False
        self.follower_stabilized = False

        self.tr = None

    def odometry_callback(self, odom: Odometry):
        self.odom = odom

    def wait_for_odometry(self):
        print("Waiting for odom...")
        while self.odom == None:
            check_ctrl_c()

    def wait_to_receive_traj_msg(self):
        print("LEADER :Waiting for traj matrix...")
        while type(self.matrix) == type(None):
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

    def land(self):
        print("Landing...")
        # Land string is not necessary, but it is nice to have
        safety_land_publisher.publish("Land")
        follower_safety_land_publisher.publish("Land")

    def follower_land(self):
        print("Follower landing...")
        # Land string is not necessary, but it is nice to have
        follower_safety_land_publisher.publish("Land")

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

    def wait_follower_to_stabilize(self):
        rate = rospy.Rate(20)
        while self.follower_stabilized_pos == None:
            check_ctrl_c()
            rate.sleep()

    def follower_stabilized_callback(self, msg: PoseStamped):
        # Follower is stabilized and then sends its position to the leader
        self.follower_stabilized_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        print("Follower stabilized at position:", self.follower_stabilized_pos)
        self.follower_stabilized = True

    def publish_sync_signal(self):
        sync_pub.publish(True)

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
        pos_pub.publish(pose)

    def land_all(self):
        self.land()
        self.follower_land()


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


def init_flight():
    print("Waiting for odometry")
    executor_pos.wait_for_odometry()

    print("Leader waiting to stabilize...")
    executor_pos.wait_to_stabilize(vel_thershold=0.02)
    print("Leader waiting Folllower to stabilize...")
    executor_pos.wait_follower_to_stabilize()

    print("Leader ready to fly!")


if __name__ == "__main__":
    rospy.init_node("Traj_Executor_Position_Controller", anonymous=True)
    # rospy.sleep(5)

    br = tf.TransformBroadcaster()

    waypoints_freq = rospy.get_param("/waypoints_freq")  # TODO:implement this in the launch file

    # get command line arguments
    cf_name = str(sys.argv[1])
    executor_id = get_executor_id(cf_name)

    print("Executor postion controller with id:", executor_id)

    # ====================== Publishers ======================
    follower_safety_land_publisher = rospy.Publisher('/cf_follower/safety_land', String, queue_size=10)
    sync_pub = rospy.Publisher('sync', Bool, queue_size=10)

    leader = TrajectoryExecutor_Position_Controller(id=0, waypoints_freq=f)  # Leader's id is 0

    # ====================== Subcribers ======================
    odometry_sub = rospy.Subscriber('/pixy/vicon/{}/{}/odom'.format(cf_name, cf_name), Odometry, leader.odometry_callback)
    follower_stab = rospy.Subscriber('/cf_follower/stabilized', PoseStamped, leader.follower_stabilized_callback)
    land_all_sub = rospy.Subscriber('/land_all', String, leader.land_all)  # TODO:implement land_all to the planning side

    init_flight()

    rate = rospy.Rate(waypoints_freq)
    while not rospy.is_shutdown():

        leader.tick()

        rate.sleep()

    rospy.spin()
