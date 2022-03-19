#!/usr/bin/env python3

import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped


from nav_msgs.msg import Path
import uav_trajectory


def publish_traj_as_path(tr: uav_trajectory.Trajectory, offset: list, pub: rospy.Publisher):
    duration = tr.duration
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.Time.now()
    path.poses = []

    dt = 0.1
    for t in np.arange(0, duration, dt):
        evaluation = tr.eval(t)
        pos, yaw = evaluation.pos, evaluation.yaw
        x, y, z = pos[0], pos[1], pos[2]
        pose = PoseStamped()
        pose.pose.position.x = x+offset[0]
        pose.pose.position.y = y+offset[1]
        pose.pose.position.z = z+offset[2]
        pose.pose.orientation.w = 1
        path.poses.append(pose)

    pub.publish(path)
