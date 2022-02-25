#!/usr/bin/env python3

import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped
import simpleaudio
import collections

try:
    from execution.msg import TrajectoryPolynomialPieceMarios
except:
    from crazyswarm.msg import TrajectoryPolynomialPieceMarios

from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import tf
import uav_trajectory
import sys

from leader_follower import trajectory_matcher_time_based


def beep():
    wave_obj = simpleaudio.WaveObject.from_wave_file(
        "/home/marios/thesis_ws/src/execution/resources/beep.wav")
    play_obj = wave_obj.play()
    play_obj.wait_done()


def handle_new_trajectory(piece_pol):
    cf_id = piece_pol.cf_id
    print("Received new trajectory with cfid:", cf_id, "...")

    if cf_id == executor_id:
        executor_pos.receive_trajectory(piece_pol)


class TrajectoryExecutor_Position_Controller:
    def __init__(self, ) -> None:
        self.odom = None

    def odometry_callback(self, odom: Odometry):
        self.odom = odom

    def wait_for_odometry(self):
        while self.odom == None:
            # print("Waiting for odom...")
            if rospy.is_shutdown():
                print(
                    'ctrl+c hit...Shutting down traj_executor_position_controller node...')
                sys.exit()

    # threshold propably needs tuning
    def wait_until_get_to_pose(self, x, y, z, yaw, pos_threshold=0.4, timeout_threshold=4):
        # Wait until the crazyflie gets to the pose-->nomrm(error) is smaller than threshold value
        # or timeout occurs (timeout_threshold)

        des_pose = PoseStamped()
        des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z = x, y, z

        error = np.inf
        t0 = rospy.get_time()
        while error > pos_threshold:
            if rospy.get_time()-t0 > timeout_threshold:
                break

            des = np.array(
                [des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z])
            actual = np.array([self.odom.pose.pose.position.x,
                              self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
            # print("Waiting to go to pose...")
            error = np.linalg.norm(des - actual)
            time.sleep(0.1)

    def receive_trajectory(self, piece_pol):
        print("Crazyflie with id:", executor_id, "received trajectory...")
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

        # 8 coeffs per x,y,z,yaw + 1 for duration
        matrix = np.zeros((lines, 1+8*4))
        matrix[:, 0] = durations.flatten()
        matrix[:, 1:9] = x
        matrix[:, 9:17] = y
        matrix[:, 17:25] = z
        matrix[:, 25:33] = yaw

        self.matrix = matrix
        self.traj_matcher = trajectory_matcher_time_based(matrix)

        file_name = "piecewise_pole_test_{}.csv".format(executor_id)
        np.savetxt(file_name,  matrix, delimiter=",", fmt='%.6f')
        tr = uav_trajectory.Trajectory()

        # TODO:Load trajectory without using file
        tr.loadcsv(file_name, skip_first_row=False)
        self.tr = tr

    def go_to_pose(self, x, y, z, yaw, offset=[0, 0, 0]):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()

        x, y, z = x+offset[0], y+offset[1], z+offset[2]  # adding offset
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q[
            0], q[1], q[2], q[3]

        pos_pub.publish(p)

    def get_traj_start_pose(self) -> PoseStamped:
        eval = self.tr.eval(0)

        start_pose = PoseStamped()
        start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z = eval.pos[
            0], eval.pos[1], eval.pos[2]
        start_pose.pose.orientation = tf.transformations.quaternion_from_euler(
            0, 0, eval.yaw)

        return start_pose

    def take_off(self, height=1):
        if self.odom == None:
            return False
        x, y, z = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z

        self.go_to_pose(x, y, height, 0)
        self.wait_until_get_to_pose(x, y, height, 0, threshold=0.4)

    def land(self):
        print("Landing...")
        # Land string is not necessary, but it is nice to have
        safety_land_publisher.publish("Land")

    def execute_trajectory(self, matrix, relative=False):
        file_name = "piecewise_pole_test_{}.csv".format(executor_id)
        np.savetxt(file_name,  matrix, delimiter=",", fmt='%.6f')
        tr = uav_trajectory.Trajectory()

        # TODO:Load trajectory without using file
        tr.loadcsv(file_name, skip_first_row=False)
        self.tr = tr
        print("duration:", tr.duration)

        rospy.sleep(3)  # sleep for 3 seconds to make sure the drone is stable

        if relative:
            # if trajectory relative to current position
            # the drone won't have to move before starting the trajector
            offset = [self.odom.pose.pose.position.x,
                      self.odom.pose.pose.position.y, self.odom.pose.pose.position.z]
            # offset = [0, 4, 1]

        else:
            # If not relative the drone has to go first at the starting position
            offset = [0, 4, 1]
            start_pose = self.get_traj_start_pose()
            print("start_pose:", start_pose.pose.position.x,
                  start_pose.pose.position.y, start_pose.pose.position.z)
            rospy.sleep(3)

            self.go_to_pose(start_pose.pose.position.x + offset[0],
                            start_pose.pose.position.y + offset[1],
                            start_pose.pose.position.z + offset[2], yaw=0)

            self.wait_until_get_to_pose(start_pose.pose.position.x, start_pose.pose.position.y,
                                        start_pose.pose.position.z, yaw=0, threshold=0.1)

        # frequency of sending references to the controller in hz
        rate = rospy.Rate(100.0)  # maybe need to increase this
        t0 = rospy.get_time()

        t = 0
        dt = 0.1
        beep()
        while not rospy.is_shutdown():
            t = t+dt
            if t > tr.duration:
                break

            evaluation = tr.eval(t)
            pos, yaw = evaluation.pos, evaluation.yaw
            x, y, z = pos[0], pos[1], pos[2]

            print("t:", t, "x:", x, "y:", y, "z:", z, "yaw:", yaw)
            self.go_to_pose(x, y, z, yaw, offset=offset)
            self.wait_until_get_to_pose(
                x+offset[0], y+offset[1], z+offset[2], yaw, threshold=0.15)

            rate.sleep()

        self.land()

    def receive_leader_pose(self, msg):
        if not self.leader_started_trajectory_flag:
            # Leader has not started the trajectory yet
            print("Leader has not started the trajectory yet")
            return 0

        if self.traj_matcher == None:
            print("traj_matcher hasn't been initialized!")
            print("Safety landing...")
            self.land()
            sys.exit(1)  # not sure if this is the right way to exit
            return

        self.leader_pose = msg
        t = self.traj_matcher.get_corresponding_t(
            self.leader_pose.pose.position)

        print("estimated t:", t)

        evaluation = self.tr.eval(t)
        pos, yaw = evaluation.pos, evaluation.yaw
        x, y, z = pos[0], pos[1], pos[2]
        print("t:", t, "  ======> Going at :" "x:",
              x, "y:", y, "z:", z, "yaw:", yaw)
        offset = [-1, 4, 1]
        self.go_to_pose(x, y, z, yaw, offset=offset)

    def leader_started_trajectory(self, msg):
        self.leader_started_trajectory_flag = True


def test_system():
    executor_pos.wait_for_odometry()
    # executor_pos.take_off(height=0.5)  # takes off and waits until it is at the desired height
    # rospy.sleep(2)
    x, y, z = 0.5, 4.5, 1.5

    executor_pos.go_to_pose(x, y, z, yaw=0)  # goes to the desired position
    # waits until it is at the desired position
    executor_pos.wait_until_get_to_pose(x, y, z, yaw=0, threshold=0.4)

    rospy.sleep(2)

    executor_pos.land()


def test_system_trajectory():
    executor_pos.wait_for_odometry()
    traj = uav_trajectory.Trajectory()

    # load trajectory file
    # traj_file_name = "/home/marios/thesis_ws/src/crazyflie_ros/crazyflie_demo/scripts/figure8.csv"

    traj_file_name = "/home/marios/thesis_ws/src/crazyflie_ros/crazyflie_demo/scripts/simple_line.csv"

    matrix = np.loadtxt(traj_file_name, delimiter=",",
                        skiprows=1, usecols=range(33)).reshape(1, 33)

    print(matrix.shape)
    # executing trajectory
    executor_pos.execute_trajectory(matrix, relative=False)

    executor_pos.land()


def test_trajectory_matcher():
    traj_file_name = "/home/marios/thesis_ws/src/crazyflie_ros/crazyflie_demo/scripts/figure8.csv"
    matrix = np.loadtxt(traj_file_name, delimiter=",",
                        skiprows=1, usecols=range(33))

    executor_pos.matrix = matrix
    executor_pos.traj_matcher = trajectory_matcher_time_based(matrix)

    tr = uav_trajectory.Trajectory()
    follower_traj_file_name = "/home/marios/thesis_ws/src/crazyflie_ros/crazyflie_demo/scripts/figure8.csv"
    tr.loadcsv(follower_traj_file_name, skip_first_row=True)
    executor_pos.tr = tr
    # t: 0.07356752525252525 x: 2.3671222828339965e-05 y: -3.8149882711119446e-05 z: 0.0 yaw: 0.0
    # t: 1.839188131313131 x: 0.9508312059227128 y: -0.21242448466646902 z: 0.0 yaw: 0.0
    # t: 1.9863231818181817 x: 0.9830702810065 y: -0.06176073650510305 z: 0.0 yaw: 0.0

    leader_poses = []
    leader_poses.append([2.3671222828339965e-05, -3.8149882711119446e-05, 0.0])
    leader_poses.append([0.9508312059227128, -0.21242448466646902, 0.0])
    leader_poses.append([0.9830702810065, -0.06176073650510305, 0.0])

    for p in leader_poses:
        # create pose stamped message
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]

        print("pose:", pose.pose.position.x,
              pose.pose.position.y, pose.pose.position.z, end=" ----->  ")

        executor_pos.receive_leader_pose(pose)

        rospy.sleep(3)


if __name__ == "__main__":
    rospy.init_node("Traj_Executor_Position_Controller", anonymous=True)

    # get command line arguments
    cf_name = str(sys.argv[1])
    common_prefix = "demo_crazyflie"
    # get id after prefix
    try:
        # if execeured via rosrun
        executor_id = int(cf_name[len(common_prefix)+2:])
    except Exception as e:
        # if executed via roslaunch
        executor_id = int(cf_name[len(common_prefix):])

    print("Executor postion controller with id:", executor_id)

    safety_land_publisher = rospy.Publisher(
        'safety_land', String, queue_size=10)
    pos_pub = rospy.Publisher('reference', PoseStamped, queue_size=10)

    print("Waiting to connect to reference topic..")
    while pos_pub.get_num_connections() < 1:
        if rospy.is_shutdown():
            sys.exit()

    print("Connected to reference topic")

    print("Got reference...")

    executor_pos = TrajectoryExecutor_Position_Controller()
    odometry_sub = rospy.Subscriber('/pixy/vicon/demo_crazyflie{}/demo_crazyflie{}/odom'.format(executor_id, executor_id),
                                    Odometry, executor_pos.odometry_callback)

    leader_sub = rospy.Subscriber(
        '/cf_leader/reference', PoseStamped, executor_pos.receive_leader_pose)

    start_traj_sub = rospy.Publisher(
        'start_trajectory', String, executor_pos.leader_started_trajectory)

    # test_system()  # Used to check the functionality of the system

    # test_system_trajectory()

    test_trajectory_matcher()

    rospy.spin()
