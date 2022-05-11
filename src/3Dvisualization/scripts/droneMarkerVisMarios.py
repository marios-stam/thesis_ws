#!/usr/bin/env python3

import rospy
from tf import TransformListener, transformations
import numpy as np
import os
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf

DRONES_NUMBER = 2


def get_executor_id(cf_name):
    # get id after prefix
    try:
        common_prefix = "demo_crazyflie"
        executor_id = int(cf_name[len(common_prefix):])
    except:
        common_prefix = "crazyflie"
        executor_id = int(cf_name[len(common_prefix):])

    return executor_id


class DroneMarker(Marker):
    def __init__(self, id, pos=[0, 0, 0], rot=[0, 0, 0, 0]):
        super().__init__()
        self.header.frame_id = "world"
        self.header.stamp = rospy.get_rostime()
        self.ns = "cf"
        self.id = id
        self.type = Marker.MESH_RESOURCE
        self.mesh_resource = "package://3Dvisualization/resources/Quadcopter.stl"
        self.action = 0

        self.updatePose(pos, rot)

        scale_fac = 1/1000
        self.scale.x = scale_fac
        self.scale.y = scale_fac
        self.scale.z = scale_fac

        self.color.r = 0.0
        self.color.g = 1.0
        self.color.b = 0.0
        self.color.a = 1.0

        self.lifetime = rospy.Duration(0)

    def updatePose(self, pos, quatern):
        self.pose.position.x = pos[0]
        self.pose.position.y = pos[1]
        self.pose.position.z = pos[2]

        rpy = transformations.euler_from_quaternion(quatern)

        quatern = transformations.quaternion_from_euler(
            rpy[0] + pi/2, rpy[1], rpy[2])

        self.pose.orientation.x = quatern[0]
        self.pose.orientation.y = quatern[1]
        self.pose.orientation.z = quatern[2]
        self.pose.orientation.w = quatern[3]


class DroneMarkersArray(MarkerArray):
    def __init__(self):
        super().__init__()
        self.markers = []
        self.id_index_dict = {}

    def update(self, id, trans, rot):

        if id not in self.id_index_dict.keys():
            print(id, "not in ", self.id_index_dict.keys())
            self.id_index_dict[id] = len(self.id_index_dict.keys())
            self.markers.append(DroneMarker(id, trans, rot))
        else:

            index = self.id_index_dict[id]
            self.markers[index].updatePose(trans, rot)


if __name__ == "__main__":

    rospy.init_node("droneMarkerVis", anonymous=True)
    listener = TransformListener()

    rate = rospy.Rate(30.0)  # hz

    dronesMarkPub = rospy.Publisher(
        'dronesMarkers_array',  MarkerArray, queue_size=10)

    drones = DroneMarkersArray()
    topic_templates = []

    for i in range(DRONES_NUMBER):
        topic_template = "/drone"
        topic_templates.append(topic_template + str(id))

    try:
        # This code works in case of online planning
        leader_cf_name = rospy.get_param("/cf_follower_name")
        follower_cf_name = rospy.get_param("/cf_leader_name")

        # get id after prefix
        leader_id = get_executor_id(leader_cf_name)
        follower_id = get_executor_id(follower_cf_name)

        # drones positions subscriber
        leader_top = 'demo_crazyflie{}/demo_crazyflie{}'.format(leader_id, leader_id)
        follower_top = 'demo_crazyflie{}/demo_crazyflie{}'.format(follower_id, follower_id)

        topic_templates.append(leader_top)
        topic_templates.append(follower_top)

    except:
        rospy.logerr("No online planning")

    while not rospy.is_shutdown():

        for id, topic in enumerate(topic_templates):
            try:
                (trans, rot) = listener.lookupTransform('/world', topic, rospy.Time(0))

                drones.update(id, trans, rot)

            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        dronesMarkPub.publish(drones)
        rate.sleep()
