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

launch3 = roslaunch.scriptapi.ROSLaunch()


def planning_finished_callback(msg):
    if not planning_finished_callback.published_once:
        print("Planning finished")

        file_name = "/home/marios/thesis_ws/src/execution/launch/formation.launch"
        launch3.parent = roslaunch.parent.ROSLaunchParent(uuid, [file_name])
        launch3.start()
        print("SEQUENCER: Launching", file_name)

        planning_finished_callback.published_once = False


planning_finished_callback.published_once = False


if __name__ == '__main__':
    """
    This script is responsible for launching the path planning before take off,
    which deems as starting positions of  the drones,their current ones with a z (height) offset.
    As soon as the path planning is finished, the drones will take off and start their trajectories.
    """

    rospy.init_node('roslaunch_sequencer')
    print("Called roslaunch_sequencer!")

    rospy.Subscriber("/finished_planning", String, planning_finished_callback)

    try:
        launch.spin()
        launch3.spin()
    finally:
        # After Ctrl+C, stop all nodes from running
        launch.shutdown()
        launch3.shutdown()
