#!/usr/bin/env python3

import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped
import simpleaudio
import collections
from nav_msgs.msg import Path
from common_functions import publish_traj_as_path, check_ctrl_c, get_executor_id
from std_msgs.msg import Bool
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

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
exec_pkg_path = rospack.get_path('execution')
drone_path_planning_path = rospack.get_path('drone_path_planning')


def beep():
    wave_obj = simpleaudio.WaveObject.from_wave_file(
        "/home/marios/thesis_ws/src/execution/resources/beep.wav")
    play_obj = wave_obj.play()
    play_obj.wait_done()


def handle_new_trajectory(piece_pol):
    cf_id = int(piece_pol.cf_id)
    print("FOLLOWER: Received new trajectory with cfid:", cf_id, "...")
    if cf_id == 1:
        executor_pos.receive_executor_trajectory(piece_pol)
    elif cf_id == 0:
        executor_pos.receive_leader_trajectory(piece_pol)


class TrajectoryExecutor_Position_Controller:
    def __init__(self, ) -> None:
        self.odom = None
        self.leader_started_trajectory_flag = False
        self.traj_matcher = None
        self.tr = None
        self.leader_synced = False

    def odometry_callback(self, odom: Odometry):
        self.odom = odom

    def wait_for_odometry(self):
        while self.odom == None:
            # print("Waiting for odom...")
            check_ctrl_c()
            rospy.sleep(0.1)

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

    def receive_executor_trajectory(self, piece_pol):
        print("FOLLOWER crazyflie with id:", executor_id, "received follower trajectory...")
        matrix = TrajectoryExecutor_Position_Controller.build_matrix_from_traj_msg(piece_pol)

        file_name = exec_pkg_path+"/resources/piecewise_pols/piecewise_pole_test_{}.csv".format(executor_id)
        np.savetxt(file_name,  matrix, delimiter=",", fmt='%.6f')
        tr = uav_trajectory.Trajectory()

        # TODO:Load trajectory without using file
        tr.loadcsv(file_name, skip_first_row=False)
        self.tr = tr

    def receive_leader_trajectory(self, piece_pol):
        # Receives the leader trajectory and initialize the trajectory matcher with it
        print("FOLLOWER crazyflie with id:", leader_id, "received leader trajectory...")
        matrix = TrajectoryExecutor_Position_Controller.build_matrix_from_traj_msg(piece_pol)
        self.traj_matcher = trajectory_matcher_time_based(matrix)

    def build_matrix_from_traj_msg(piece_pol):
        lines = int(len(piece_pol.poly_x)/8)

        print(len(piece_pol.poly_x))

        x = np.array(piece_pol.poly_x).reshape((lines, 8))
        y = np.array(piece_pol.poly_y).reshape((lines, 8))
        z = np.array(piece_pol.poly_z).reshape((lines, 8))
        yaw = np.array(piece_pol.poly_yaw).reshape((lines, 8))
        durations = np.array(piece_pol.durations).reshape((lines, 1))

        # 8 coeffs per x,y,z,yaw + 1 for duration
        matrix = np.zeros((lines, 1+8*4))
        matrix[:, 0] = durations.flatten()
        matrix[:, 1:9] = x
        matrix[:, 9:17] = y
        matrix[:, 17:25] = z
        matrix[:, 25:33] = yaw

        return matrix

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
            rospy.logerr("No leader odometry received yet")
            sys.exit()

        x, y, z = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z

        # self.go_to_pose(x, y, height, 0)
        self.wait_until_get_to_pose(x, y, height, 0, pos_threshold=0.05, timeout_threshold=4)

    def land(self):
        print("Landing...")
        # Land string is not necessary, but it is nice to have
        safety_land_publisher.publish("Land")

    def receive_leader_pose(self, msg):
        if self.leader_started_trajectory_flag == False:
            # Leader has not started the trajectory yet
            print("FOLLOWER:Leader has not started the trajectory yet")
            return 0

        if self.traj_matcher == None or self.tr == None:
            print("traj_matcher or self.tr hasn't been initialized!")
            print("Safety landing...")
            self.land()
            sys.exit(1)  # not sure if this is the right way to exit
            return

        self.leader_pose = msg
        try:
            t = self.traj_matcher.get_corresponding_t(self.leader_pose.pose.position)
        except Exception as e:
            print("Error in getting corresponding t:", str(e))
            return

        evaluation = self.tr.eval(t)
        pos, yaw = evaluation.pos, evaluation.yaw
        x, y, z = pos[0], pos[1], pos[2]
        # print("Follower:", "t:", t, "  ======> Going at :" "x:", x, "y:", y, "z:", z, "yaw:", yaw)

        offset = [0, 0, 0]
        self.go_to_pose(x, y, z, yaw, offset=offset)
        # TODO: maybe need to wait until get to pose

    def leader_started_trajectory(self, msg):
        print("Leader started the trajectory")
        self.leader_started_trajectory_flag = True

    def wait_to_stabilize(self, vel_thershold=0.04):
        # wait until the drone is stabilized (velocity magnitude below threshold)

        stabilized = False
        altitude = 0
        rate = rospy.Rate(20)

        while (not stabilized) or (altitude < 0.3):
            check_ctrl_c()
            # get velocities
            velocities = [self.odom.twist.twist.linear.x,
                          self.odom.twist.twist.linear.y]

            altitude = self.odom.pose.pose.position.z

            stabilized = all(abs(vel) < vel_thershold for vel in velocities)
            rate.sleep()
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = self.odom.pose.pose.position
        stabilized_pos_pub.publish(pose)

    def wait_for_leader_sync(self):
        # wait until the leader sends sync signal
        while not self.leader_synced:
            check_ctrl_c()

    def leader_sync_callback(self, msg: Bool):
        print("Received sync signal from leader")
        self.leader_synced = msg.data

    def go_to_traj_start_pos(self, offset=[0, 0, 0]):
        # Go to start position
        if self.tr == None:
            rospy.logerr("No trajectory received yet")
            sys.exit()

        pos = self.tr.eval(0.0).pos
        print("Follower going to pose: {} with offset: {}".format(pos, offset))

        executor_pos.go_to_pose(pos[0], pos[1], pos[2], yaw=0, offset=offset)


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


def test_traj_matcher_general():
    leader_traj_file = rospy.get_param("/cf_leader_traj")
    follower_traj_file = rospy.get_param("/cf_follower_traj")
    print("leader_traj_file:", leader_traj_file)
    print("follower_traj_file:", follower_traj_file)

    # Load leader trajectory
    leader_matrix = np.loadtxt(leader_traj_file, delimiter=",", skiprows=0, usecols=range(33))

    if leader_matrix.shape[0] == 33:
        leader_matrix = leader_matrix.reshape(1, 33)

    executor_pos.matrix = leader_matrix
    executor_pos.traj_matcher = trajectory_matcher_time_based(leader_matrix)

    # Load follower trajectory
    follower_tr = uav_trajectory.Trajectory()
    follower_tr.loadcsv(follower_traj_file, skip_first_row=False)
    executor_pos.tr = follower_tr

    # wait until stabilize after take off TODO:make this more robust
    executor_pos.wait_to_stabilize()

    executor_pos.wait_for_leader_sync()
    print("FOLLOWER:Received SYNC signal...")
    # Go to start position
    pos = follower_tr.eval(0.0).pos
    print("Follower going to pose:", pos)

    offset = [0, 0, 0]
    publish_traj_as_path(follower_tr, offset, path_pub)
    executor_pos.go_to_pose(pos[0], pos[1], pos[2], yaw=0, offset=[0, 0, 0])


def use_already_generated_trajectories():
    leader_traj_file = rospy.get_param("/cf_leader_traj")
    follower_traj_file = rospy.get_param("/cf_follower_traj")
    print("FOLLOWER: leader_traj_file:", leader_traj_file)
    print("FOLLOWER: follower_traj_file:", follower_traj_file)

    # Load leader trajectory
    leader_matrix = np.loadtxt(leader_traj_file, delimiter=",", skiprows=0, usecols=range(33))
    executor_pos.traj_matcher = trajectory_matcher_time_based(leader_matrix)

    # Load follower trajectory
    follower_tr = uav_trajectory.Trajectory()
    follower_tr.loadcsv(follower_traj_file, skip_first_row=False)
    executor_pos.tr = follower_tr


def planning_before_take_off():
    prefix_path = drone_path_planning_path+"/resources/trajectories/"
    leader_traj_file = prefix_path + "Pol_matrix_leader.csv"
    follower_traj_file = prefix_path + "Pol_matrix_follower.csv"

    print("leader_traj_file:", leader_traj_file)
    print("follower_traj_file:", follower_traj_file)

    # Load leader trajectory
    leader_matrix = np.loadtxt(leader_traj_file, delimiter=",", skiprows=0, usecols=range(33))

    if leader_matrix.shape[0] == 33:
        leader_matrix = leader_matrix.reshape(1, 33)

    executor_pos.matrix = leader_matrix
    executor_pos.traj_matcher = trajectory_matcher_time_based(leader_matrix)

    # Load follower trajectory
    follower_tr = uav_trajectory.Trajectory()
    follower_tr.loadcsv(follower_traj_file, skip_first_row=False)
    executor_pos.tr = follower_tr

    use_already_generated_trajectories()  # TODO: remove this in real flight

    # wait until stabilize after take off
    executor_pos.wait_to_stabilize()

    executor_pos.wait_for_leader_sync()
    print("FOLLOWER:Received SYNC signal...")

    # Go to start position
    pos = executor_pos.tr.eval(0.0).pos
    print("Follower going to start pose:", pos)

    offset = [0, 0, 0]
    publish_traj_as_path(executor_pos.tr, offset, path_pub)
    executor_pos.go_to_pose(pos[0], pos[1], pos[2], yaw=0, offset=[0, 0, 0])


def live_planning():
    # wait until stabilize after take off
    print("FOLLOWER:Waiting to stabilize...")
    executor_pos.wait_to_stabilize()
    print("FOLLOWER:Stabilized...")

    executor_pos.wait_for_leader_sync()
    print("FOLLOWER:Received SYNC signal...")

    print("FOLLOWER:Going to start position...")
    executor_pos.go_to_traj_start_pos(offset=[0, 0, 0])


if __name__ == "__main__":
    rospy.init_node("Traj_Executor_Position_Controller", anonymous=True)
    rospy.sleep(5)
    # get command line arguments
    cf_name = str(sys.argv[1])
    leader_cf_name = str(sys.argv[2])

    # get id after prefix
    executor_id = get_executor_id(cf_name)
    leader_id = get_executor_id(leader_cf_name)

    print("FOLLOWER:Executor follower position controller with id:", executor_id)
    print("FOLLOWER:Leader position controller with id:", leader_id)

    safety_land_publisher = rospy.Publisher('safety_land', String, queue_size=10)
    pos_pub = rospy.Publisher('reference', PoseStamped, queue_size=10)

    print("FOLLOWER:Waiting to connect to reference topic..")
    while pos_pub.get_num_connections() < 1:
        if rospy.is_shutdown():
            sys.exit()

    print("FOLLOWER:Connected to reference topic")
    executor_pos = TrajectoryExecutor_Position_Controller()
    odom_topic = '/pixy/vicon/demo_crazyflie{}/demo_crazyflie{}/odom'.format(executor_id, executor_id)
    print("FOLLOWER: odom_topic:", odom_topic)
    odometry_sub = rospy.Subscriber(odom_topic, Odometry, executor_pos.odometry_callback)

    leader_sub = rospy.Subscriber('/cf_leader/reference', PoseStamped, executor_pos.receive_leader_pose)

    start_traj_sub = rospy.Subscriber('/cf_leader/start_trajectory', String, executor_pos.leader_started_trajectory)

    # Subscribe to trajectory topic and then execute it after going to the first waypoint
    rospy.Subscriber('/piece_pol', TrajectoryPolynomialPieceMarios, handle_new_trajectory)
    path_pub = rospy.Publisher('/cf_follower/path', Path, queue_size=10)

    # wait until stabilize after take off and then send its position
    stabilized_pos_pub = rospy.Publisher('stabilized', PoseStamped, queue_size=10)

    sync_sub = rospy.Subscriber('/cf_leader/sync', Bool, executor_pos.leader_sync_callback)

    # Wait for odometry
    print("FOLLOWER:Waiting for odometry to be ready..")
    executor_pos.wait_for_odometry()

    planning_time = rospy.get_param("/planning_time")
    # test_traj_matcher_general()
    if planning_time == "before_take_off":
        print("FOLLOWER:Planning before take off")
        planning_before_take_off()
    elif planning_time == "live":
        print("FOLLOWER:Live planning")
        live_planning()
    else:
        rospy.logerr("Invalid planning time:", planning_time)
        sys.exit()

    rospy.spin()
