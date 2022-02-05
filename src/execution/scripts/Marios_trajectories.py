#!/usr/bin/env python3

import rospy
from rospy.client import INFO
from tf import TransformListener
import numpy as np

from pycrazyswarm.crazyflie import Crazyflie
from uav_trajectory import Trajectory
from crazyswarm.msg import TrajectoryPolynomialPieceMarios

import os
import uav_trajectory


def handle_new_trajectory(piece_pol: TrajectoryPolynomialPieceMarios):
    cf_id = piece_pol.cf_id
    print("Received new trajectory with cfid:", cf_id, "...")
    cfs_traj_executors[cf_id].receive_trajectory(piece_pol)


def callback(piece_pol: TrajectoryPolynomialPieceMarios):
    print("Callback")
    cfid = piece_pol.cf_id
    print("cfid:", cfid)

    lines = int(len(piece_pol.poly_x)/8)

    print(len(piece_pol.poly_x))
    # input()
    x = np.array(piece_pol.poly_x).reshape((lines, 8))
    y = np.array(piece_pol.poly_y).reshape((lines, 8))
    z = np.array(piece_pol.poly_z).reshape((lines, 8))
    yaw = np.array(piece_pol.poly_yaw).reshape((lines, 8))
    durations = np.array(piece_pol.durations).reshape((lines, 1))

    print("x:", x.shape)
    print("y:", y.shape)
    print("z:", z.shape)
    print("yaw:", yaw.shape)
    print("durations:", durations.shape)

    matrix = np.zeros((lines, 1+8*4))  # 8 coeffs per x,y,z,yaw + 1 for duration
    matrix[:, 0] = durations.flatten()
    matrix[:, 1:9] = x
    matrix[:, 9:17] = y
    matrix[:, 17:25] = z
    matrix[:, 25:33] = yaw

    execute_trajectory(matrix)


class TrajectoryExecutor:
    def __init__(self, cf: Crazyflie) -> None:
        self.cf = cf

    def receive_trajectory(self, piece_pol: TrajectoryPolynomialPieceMarios):
        print("Crazyflie with id:", self.cf.id, "received trajectory...")
        cfid = piece_pol.cf_id

        lines = int(len(piece_pol.poly_x)/8)

        print(len(piece_pol.poly_x))

        x = np.array(piece_pol.poly_x).reshape((lines, 8))
        y = np.array(piece_pol.poly_y).reshape((lines, 8))
        z = np.array(piece_pol.poly_z).reshape((lines, 8))
        yaw = np.array(piece_pol.poly_yaw).reshape((lines, 8))
        durations = np.array(piece_pol.durations).reshape((lines, 1))

        print("x:", x.shape)
        print("y:", y.shape)
        print("z:", z.shape)
        print("yaw:", yaw.shape)
        print("durations:", durations.shape)

        matrix = np.zeros((lines, 1+8*4))  # 8 coeffs per x,y,z,yaw + 1 for duration
        matrix[:, 0] = durations.flatten()
        matrix[:, 1:9] = x
        matrix[:, 9:17] = y
        matrix[:, 17:25] = z
        matrix[:, 25:33] = yaw

        self.matrix = matrix
        self.execute_trajectory(matrix)

    def execute_trajectory(self, matrix):
        cf = self.cf
        file_name = "piecewise_pole.csv"
        names = ["duration",
                 "x^0", "x^1", "x^2", "x^3", "x^4", "x^5", "x^6", "x^7",
                 "y^0", "y^1", "y^2", "y^3", "y^4", "y^5", "y^6", "y^7",
                 "z^0", "z^1", "z^2", "z^3", "z^4", "z^5", "z^6", "z^7",
                 "yaw^0", "yaw^1", "yaw^2", "yaw^3", "yaw^4", "yaw^5", "yaw^6", "yaw^7"]

        np.savetxt(file_name,  matrix, delimiter=",", fmt='%.6f')
        # file_name = "/home/marios/crazyswarm/ros_ws/src/crazyswarm/scripts/figure8.csv"
        traj = uav_trajectory.Trajectory()
        traj.loadcsv(file_name)  # TODO:Loaf trajectory without using file

        cf.uploadTrajectory(trajectoryId=0, pieceOffset=0, trajectory=traj)

        print("Taking off...")
        cf.takeoff(targetHeight=1, duration=4.0)
        rospy.sleep(4)

        print("Going to start position...")
        evaluation = traj.eval(t=0)
        pos, yaw = evaluation.pos, evaluation.yaw
        x, y, z = pos[0], pos[1], pos[2]

        print("x, y, z, yaw:", x, y, z, yaw)
        cf.goTo([x, y, z], yaw=0, duration=4.0)
        rospy.sleep(4)

        print("Starting trajectory...")
        cf.startTrajectory(trajectoryId=0)
        rospy.sleep(traj.duration)

        print("Landing...")
        cf.land(targetHeight=0.02, duration=2.0)
        rospy.sleep(2)

        cf.cmdStop()


if __name__ == "__main__":
    rospy.init_node("CrazyflieDistributed")

    cf = None
    cfs_traj_executors = []  # list of TrajectoryExecutor objects
    for crazyflie in rospy.get_param("crazyflies"):
        cfid = int(crazyflie["id"])

        initialPosition = crazyflie["initialPosition"]
        tf_listener = TransformListener()
        cf = Crazyflie(cfid, initialPosition, tf_listener)
        print("Found cf with id:", cfid)
        cfs_traj_executors.append(TrajectoryExecutor(cf))

    rospy.Subscriber('piece_pol', TrajectoryPolynomialPieceMarios, handle_new_trajectory)

    rospy.spin()
