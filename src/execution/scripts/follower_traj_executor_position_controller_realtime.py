#!/usr/bin/env python3

import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped
import simpleaudio
import collections
from nav_msgs.msg import Path
from common_functions import publish_traj_as_path, check_ctrl_c, get_executor_id
from std_msgs.msg import Bool
from traj_executor_position_controller_realtime_base import TrajectoryExecutor_Position_Controller_Realtime_Base
try:
    from execution.msg import TrajectoryPolynomialPieceMarios
except:
    from crazyswarm.msg import TrajectoryPolynomialPieceMarios

from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import tf
import uav_trajectory
import sys

from leader_follower import trajectory_matcher_time_based
from traj_executor_position_controller_realtime_base import TrajectoryExecutor_Position_Controller_Realtime_Base, beep

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
exec_pkg_path = rospack.get_path('execution')
drone_path_planning_path = rospack.get_path('drone_path_planning')


class TrajectoryExecutor_Position_Controller_Follower(TrajectoryExecutor_Position_Controller_Realtime_Base):
    def __init__(self, cf_id: int, waypoints_freq: float) -> None:
        super().__init__(cf_id, waypoints_freq)

        # Follower specific code
        self.leader_started_trajectory_flag = False
        self.leader_synced = False
        self.traj_matcher = None

    def receive_leader_trajectory(self, piece_pol):
        # Receives the leader trajectory and initialize the trajectory matcher with it
        print("FOLLOWER crazyflie with id:", leader_id, "received leader trajectory...")
        matrix = TrajectoryExecutor_Position_Controller.build_matrix_from_traj_msg(piece_pol)
        self.traj_matcher = trajectory_matcher_time_based(matrix)

    def receive_leader_pose(self, msg):
        if self.leader_started_trajectory_flag == False:
            # Leader has not started the trajectory yet
            print("Leader has not started the trajectory yet")
            return 0

        if self.traj_matcher == None or self.tr == None:
            print("traj_matcher or self.tr hasn't been initialized!")
            print("Safety landing...")
            self.land()
            sys.exit(1)  # not sure if this is the right way to exit
            return

        self.leader_pose = msg
        try:
            t = self.traj_matcher.get_corresponding_t(self.leader_pose.pose.position)
        except:
            print("Error in getting corresponding t")
            return

        evaluation = self.tr.eval(t)
        pos, yaw = evaluation.pos, evaluation.yaw
        x, y, z = pos[0], pos[1], pos[2]
        # print("Follower:", "t:", t, "  ======> Going at :" "x:", x, "y:", y, "z:", z, "yaw:", yaw)

        offset = [0, 0, 0]
        self.go_to_pose(x, y, z, yaw, offset=offset)
        # TODO: maybe need to wait until get to pose

    def leader_started_trajectory(self, msg):
        print("Leader started the trajectory")
        self.leader_started_trajectory_flag = True

    def wait_for_leader_sync(self):
        # wait until the leader sends sync signal
        while not self.leader_synced:
            check_ctrl_c()

    def leader_sync_callback(self, msg: Bool):
        print("Received sync signal from leader")
        self.leader_synced = msg.data


def follower_traj_callback(piece_pol: TrajectoryPolynomialPieceMarios):

    if piece_pol.cf_id == 1:
        print("FOLLOWER:Received trajectory...")
        follower.receive_trajectory(piece_pol)


def init_flight():
    print("FOLLOWER:Waiting for odometry...")
    follower.wait_for_odometry()

    print("FOLLOWER:Waiting controller to connect...")
    follower.wait_controller_to_connect()

    print("FOLLOWER:Waiting to stabilize...")
    follower.wait_to_stabilize()

    print("FOLLOWER:Publishing stabilized pose...")
    follower.publish_stabilized()

    print("FOLLOWER:Waiting to receive SYNC signal...")
    follower.wait_for_leader_sync()


if __name__ == "__main__":
    rospy.init_node("Traj_Executor_Position_Controller", anonymous=True)
    # get command line arguments
    cf_name = str(sys.argv[1])
    leader_cf_name = str(sys.argv[2])

    # get id after prefix
    executor_id = get_executor_id(cf_name)
    leader_id = get_executor_id(leader_cf_name)

    print("Executor follower position controller with id:", executor_id)
    print("Leader position controller with id:", leader_id)

    waypoints_freq = rospy.get_param("/waypoints_freq")  # TODO:implement this in the launch file

    follower = TrajectoryExecutor_Position_Controller_Follower(executor_id, waypoints_freq=waypoints_freq)

    # ====================== Publishers ======================

    # ====================== Subcribers ======================
    odom_topic = '/pixy/vicon/{}/{}/odom'.format(cf_name, cf_name)
    print("odom_topic:", odom_topic)
    odometry_sub = rospy.Subscriber(odom_topic, Odometry, follower.odometry_callback)

    start_traj_sub = rospy.Subscriber('/cf_leader/start_trajectory', String, follower.leader_started_trajectory)
    # leader_sub = rospy.Subscriber('/cf_leader/reference', PoseStamped, follower.receive_leader_pose)

    rospy.Subscriber('/piece_pol', TrajectoryPolynomialPieceMarios, follower_traj_callback)

    sync_sub = rospy.Subscriber('/cf_leader/sync', Bool, follower.leader_sync_callback)

    init_flight()
    print("FOLLOWER:Ready to fly!")
    beep()

    rate = rospy.Rate(waypoints_freq)
    while not rospy.is_shutdown():

        print("FOLLOWER:", end=" ")
        follower.tick()

        rate.sleep()

    rospy.spin()
