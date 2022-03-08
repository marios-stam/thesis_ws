#!/usr/bin/env python3

import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped


from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import tf
import uav_trajectory
import sys


if __name__ == "__main__":
    rospy.init_node("land_all_drones", anonymous=True)

    land_pubs = []
    land_pubs.append(rospy.Publisher(
        "/cf_leader/safety_land", String, queue_size=10))
    land_pubs.append(rospy.Publisher(
        "/cf_follower/safety_land", String, queue_size=10))

    # wait for the safety_land topics to get connections
    connected = False
    while not connected and not rospy.is_shutdown():
        for pub in land_pubs:
            if pub.get_num_connections() > 0:
                print("Connected to safety_land topic")
                connected = True
                break
        else:
            print("Waiting for safety_land topic to get connections")
            time.sleep(0.1)

    rate = rospy.Rate(50)  # hz
    while not rospy.is_shutdown():
        for pub in land_pubs:
            pub.publish("land")
        rate.sleep()
    rospy.spin()
