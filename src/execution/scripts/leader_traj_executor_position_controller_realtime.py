#!/usr/bin/env python3

from math import atan2
from sympy import re
import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped
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


def handle_new_trajectory(piece_pol):
    drone1.pol_traj_callback(piece_pol)
    drone2.pol_traj_callback(piece_pol)


class Drone_transform_updater:
    RESET_DISTANCE_THRESHOLD = 2

    def __init__(self, id, dt):
        self.id = id
        self.tr: uav_trajectory.Trajectory = None
        self.t = 0
        self.new_traj_received = False
        self.dt = dt

    def update_trajectory(self, tr: uav_trajectory.Trajectory):
        first_pos = np.array(tr.eval(0).pos)

        # If it is the first time it receives traj set prev_pos to init pos
        if self.tr == None:
            self.prev_pos = first_pos

        # distance from current pos to first_pos of received trajectory
        dist_from_curr_pos = np.linalg.norm(first_pos-self.prev_pos)
        if (dist_from_curr_pos > self.RESET_DISTANCE_THRESHOLD):
            print("opa teliosame")
            self.prev_pos = first_pos

        self.tr = tr
        self.first_tr_received = True
        self.t = 0.6

    def update_time(self):
        if self.tr == None:
            print("No trajectory received yet...")
            return

        self.t += self.dt
        if self.t >= self.tr.duration:
            self.t = self.tr.duration

    def tick(self):
        if self.tr == None:
            print("No trajectory received yet...")
            return

        self.update_time()

        pos = np.array(self.tr.eval(self.t).pos)

        delta_pos = pos-self.prev_pos

        MAX_DELTA = 0.5
        delta_mag = np.linalg.norm(delta_pos)
        delta_pos = delta_pos if delta_mag < MAX_DELTA else delta_pos/delta_mag*MAX_DELTA

        pos = self.prev_pos + dt*delta_pos

        # publish transform
        print("Publishing transform for drone {} wits pos {}".format(self.id+1, pos))
        br.sendTransform(pos, [0, 0, 0, 1], rospy.Time.now(), "drone"+str(self.id+1), "world")
        self.prev_pos = pos

    def pol_traj_callback(self, piece_pol):
        if piece_pol.cf_id == self.id:
            tr = receive_trajectory(piece_pol)
            self.update_trajectory(tr)


def receive_trajectory(piece_pol):
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


if __name__ == "__main__":
    rospy.init_node("Traj_Executor_Position_Controller", anonymous=True)
    # rospy.sleep(5)

    br = tf.TransformBroadcaster()

    pos_pub = rospy.Publisher('reference', PoseStamped, queue_size=10)
    f = 50
    dt = 1/f*1.3

    drone1 = Drone_transform_updater(id=0, dt=dt)
    drone2 = Drone_transform_updater(id=1, dt=dt)

    rospy.Subscriber('/piece_pol', TrajectoryPolynomialPieceMarios, handle_new_trajectory)

    rate = rospy.Rate(f)
    while not rospy.is_shutdown():

        drone1.tick()
        drone2.tick()

        rate.sleep()

    rospy.spin()
