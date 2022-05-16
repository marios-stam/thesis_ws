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
from traj_executor_position_controller_realtime_base import TrajectoryExecutor_Position_Controller_Realtime_Base, beep

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
exec_pkg_path = rospack.get_path('execution')


class TrajectoryExecutor_Position_Controller_Leader(TrajectoryExecutor_Position_Controller_Realtime_Base):
    def __init__(self, cf_id: int, waypoints_freq: float, new_ref_dist_threshold: float) -> None:
        super().__init__(cf_id, waypoints_freq, new_ref_dist_threshold)

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


def leader_traj_callback(piece_pol: TrajectoryPolynomialPieceMarios):

    if piece_pol.cf_id == 0:
        print("LEADER:Received trajectory...")
        leader.receive_trajectory(piece_pol)


def init_flight():
    print("LEADER:Waiting for odometry")
    leader.wait_for_odometry()

    print("LEADER:Waiting controller to connect...")
    leader.wait_controller_to_connect()

    print("LEADER:Waiting to stabilize...")
    leader.wait_to_stabilize(vel_thershold=0.04)

    print("LEADER:Waiting Folllower to stabilize...")
    leader.wait_follower_to_stabilize()

    print("LEADER:Publishing SYNC signal...")
    leader.publish_sync_signal()


if __name__ == "__main__":
    rospy.init_node("Traj_Executor_Position_Controller", anonymous=True)
    # rospy.sleep(5)

    br = tf.TransformBroadcaster()

    waypoints_freq = rospy.get_param("/waypoints_freq")  # TODO:implement this in the launch file

    # get command line arguments
    cf_name = str(sys.argv[1])
    executor_id = get_executor_id(cf_name)

    print("Executor postion controller with id:", executor_id)

    waypoints_freq = rospy.get_param("/waypoints_freq")
    new_ref_dist_threshold = rospy.get_param("/new_ref_dist_threshold")

    leader = TrajectoryExecutor_Position_Controller_Leader(
        cf_id=0, waypoints_freq=waypoints_freq, new_ref_dist_threshold=new_ref_dist_threshold)  # Leader's id is 0

    # ====================== Publishers ======================
    follower_safety_land_publisher = rospy.Publisher('/cf_follower/safety_land', String, queue_size=10)
    sync_pub = rospy.Publisher('sync', Bool, queue_size=10)

    # ====================== Subcribers ======================
    odom_topic = '/pixy/vicon/{}/{}/odom'.format(cf_name, cf_name)
    print("odom_topic:", odom_topic)

    odometry_sub = rospy.Subscriber(odom_topic, Odometry, leader.odometry_callback)
    follower_stab = rospy.Subscriber('/cf_follower/stabilized', PoseStamped, leader.follower_stabilized_callback)
    land_all_sub = rospy.Subscriber('/land_all', String, leader.land_all)  # TODO:implement land_all to the planning side

    rospy.Subscriber('/piece_pol', TrajectoryPolynomialPieceMarios, leader_traj_callback)

    init_flight()
    print("LEADER:Ready to fly!")
    beep()

    rate = rospy.Rate(waypoints_freq)
    while not rospy.is_shutdown():

        print("LEADER:", end=" ")
        leader.tick()

        rate.sleep()

    rospy.spin()
