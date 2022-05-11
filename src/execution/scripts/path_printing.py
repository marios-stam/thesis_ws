#!/usr/bin/env python3
from itertools import count
from nav_msgs.msg import Path
import rospy


from execution.msg import TrajectoryPolynomialPieceMarios
from uav_trajectory import Trajectory
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from matplotlib import pyplot as plt

POINTS_PER_TRAJ = 50


def path1_callback(data: Path):
    if (path1_callback.counter == 0):
        for pos in data.poses:
            print("{},{},{}".format(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z))

    path1_callback.counter += 1


path1_callback.counter = 0

if __name__ == "__main__":
    print("Starting Trajectory Visualizer")
    rospy.init_node("path_print")
    print("started")

    rospy.Subscriber('/drone1Path', Path, path1_callback)

    rospy.spin()
