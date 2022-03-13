#!/usr/bin/env python3

import sys
import rospy
from tf import TransformListener, transformations
import numpy as np
import os
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf
from geometry_msgs.msg import PoseStamped


def callback_leader(msg: PoseStamped):
    global leader_ref
    leader_ref = [msg.pose.position.x,
                  msg.pose.position.y, msg.pose.position.z]


def callback_follower(msg: PoseStamped):
    global follower_ref
    follower_ref = [msg.pose.position.x,
                    msg.pose.position.y, msg.pose.position.z]


if __name__ == "__main__":

    rospy.init_node("error_calc", anonymous=True)
    listener = TransformListener()

    leader_error_pub = rospy.Publisher(
        "leader_error", PoseStamped, queue_size=1)
    follower_error_pub = rospy.Publisher(
        "follower_error", PoseStamped, queue_size=1)

    leader_ref = None
    follower_ref = None

    leader_topic = rospy.get_param("/cf_leader_name")
    follower_topic = rospy.get_param("/cf_follower_name")

    leader_topic = "/{}/{}".format(leader_topic, leader_topic)
    follower_topic = "/{}/{}".format(follower_topic, follower_topic)

    drone_topics = [leader_topic, follower_topic]
    print("leader_topic:", leader_topic)
    print("follower_topic:", follower_topic)

    sub_ref_leader = rospy.Subscriber(
        'cf_leader/reference', PoseStamped, callback_leader)

    sub_ref_follower = rospy.Subscriber(
        'cf_follower/reference', PoseStamped, callback_follower)

    lead_err = PoseStamped()
    lead_err.header.frame_id = "world"
    lead_err.header.stamp = rospy.get_rostime()

    foll_err = PoseStamped()
    foll_err.header.frame_id = "world"
    foll_err.header.stamp = rospy.get_rostime()

    rate = rospy.Rate(30)  # hz
    while not rospy.is_shutdown():
        try:
            if leader_ref is not None:
                leader_real = listener.lookupTransform(
                    '/world', leader_topic, rospy.Time(0))[0]
                error = np.array(leader_ref) - np.array(leader_real)
                leader_error_norm = np.linalg.norm(error)

                lead_err.pose.position.x = error[0]
                lead_err.pose.position.y = error[1]
                lead_err.pose.position.z = error[2]

                lead_err.pose.orientation.w = leader_error_norm

                leader_error_pub.publish(lead_err)
                print("leader_error:", leader_error_norm)
            if follower_ref is not None:
                leader_error_norm = np.linalg.norm(error)

                follower_real = listener.lookupTransform(
                    '/world', follower_topic, rospy.Time(0))[0]
                error = np.array(follower_ref) - np.array(follower_real)
                follower_error_norm = np.linalg.norm(error)

                foll_err.pose.position.x = error[0]
                foll_err.pose.position.y = error[1]
                foll_err.pose.position.z = error[2]

                foll_err.pose.orientation.w = leader_error_norm

                follower_error_pub.publish(foll_err)
        except:
            pass

        rate.sleep()
