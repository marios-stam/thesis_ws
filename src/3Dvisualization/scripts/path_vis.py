#!/usr/bin/env python3

import rospy
from tf import TransformListener, transformations
import numpy as np
import os
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf


if __name__ == "__main__":

    rospy.init_node("path_vis", anonymous=True)
    listener = TransformListener()

    rate = rospy.Rate(30.0)  # hz

    path1_pub = rospy.Publisher('path1',  MarkerArray, queue_size=10)

    # get command line arguments
    leader_topic = rospy.get_param("/cf_leader_name")
    follower_topic = rospy.get_param("/cf_follower_name")

    while not rospy.is_shutdown():

        for tf_topic in drone_topics:
            try:
                (trans, rot) = listener.lookupTransform(
                    '/world', tf_topic, rospy.Time(0))

                drones.update(tf_topic, trans, rot)

            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        dronesMarkPub.publish(drones)
        rate.sleep()
