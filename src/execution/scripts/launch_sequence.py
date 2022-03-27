#!/usr/bin/env python3
from math import atan2

from rospy import Subscriber
import rospy
import roslaunch
from nav_msgs.msg import Odometry
from common_functions import get_leader_follower_names
from drone_path_planning.msg import planning_state
from std_msgs.msg import String

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.scriptapi.ROSLaunch()
file_name = "/home/marios/thesis_ws/src/execution/launch/path_planning.launch"
launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [file_name])
launch.start()

launch2 = roslaunch.scriptapi.ROSLaunch()
launch3 = roslaunch.scriptapi.ROSLaunch()


def planning_finished_callback(msg):
    if not planning_finished_callback.published_once:
        print("Planning finished")
        file_name = "/home/marios/thesis_ws/src/crazyflie_ros/crazyflie_demo/launch/crazyflie_2.launch"
        launch2.parent = roslaunch.parent.ROSLaunchParent(uuid, [file_name])
        launch2.start()

        file_name = "/home/marios/thesis_ws/src/execution/launch/formation.launch"
        launch3.parent = roslaunch.parent.ROSLaunchParent(uuid, [file_name])
        launch3.start()

        planning_finished_callback.published_once = False


planning_finished_callback.published_once = False


if __name__ == '__main__':
    rospy.init_node('roslaunch_sequencer')

    rospy.Subscriber("/finished_planning", String, planning_finished_callback)

    try:
        launch.spin()
        launch2.spin()
        launch3.spin()
    finally:
        # After Ctrl+C, stop all nodes from running
        launch.shutdown()
        launch2.shutdown()
        launch3.shutdown()
