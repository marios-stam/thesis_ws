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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# import PoseStamped

import tf


br = tf.TransformBroadcaster()


def leader_callback(data: PoseStamped):
    pos = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
    q = (0, 0, 0, 1)
    br.sendTransform(pos, q, rospy.Time.now(), "leader_ref", "world")


def follower_callback(data: PoseStamped):
    pos = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
    q = (0, 0, 0, 1)
    br.sendTransform(pos, q, rospy.Time.now(), "follower_ref", "world")


if __name__ == "__main__":
    rospy.init_node("references_vis")
    print("started")

    leader_topic = "/cf_leader/ref"
    follower_topic = "/cf_follower/ref"

    rospy.Subscriber(leader_topic, PoseStamped, leader_callback)
    rospy.Subscriber(follower_topic, PoseStamped, follower_callback)

    rospy.spin()
