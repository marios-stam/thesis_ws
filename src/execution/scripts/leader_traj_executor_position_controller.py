#!/usr/bin/env python3

from math import atan2
import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped
import simpleaudio
from nav_msgs.msg import Path
from common_functions import publish_traj_as_path, check_ctrl_c, get_executor_id
from std_msgs.msg import Bool
from drone_path_planning.msg import planning_state

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

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
exec_pkg_path = rospack.get_path('execution')


def beep():
    wave_obj = simpleaudio.WaveObject.from_wave_file(
        "/home/marios/thesis_ws/src/execution/resources/beep.wav")
    play_obj = wave_obj.play()
    play_obj.wait_done()


def handle_new_trajectory(piece_pol):
    cf_id = piece_pol.cf_id
    print("Received new trajectory with cfid:", cf_id, "...")

    if cf_id == 0:
        executor_pos.receive_trajectory(piece_pol)


class TrajectoryExecutor_Position_Controller:
    def __init__(self, ) -> None:
        self.odom = None
        self.follower_stabilized_pos = None

        self.leader_stabilized = False
        self.follower_stabilized = False

        self.matrix = None
        self.traj_started = False

    def odometry_callback(self, odom: Odometry):
        self.odom = odom

    def wait_for_odometry(self):
        print("Waiting for odom...")
        while self.odom == None:
            check_ctrl_c()

    def wait_to_receive_traj_msg(self):
        print("LEADER :Waiting for traj matrix...")
        while type(self.matrix) == type(None):
            check_ctrl_c()
            rospy.sleep(0.1)

    # threshold propably needs tuning
    def wait_until_get_to_pose(self, x, y, z, yaw, threshold=0.4, timeout_threshold=np.inf):
        # Wait until the crazyflie gets to the pose-->nomrm(error) is smaller than threshold value
        # if self.odom is None:
        #     raise Exception("No odometry received yet")

        des_pose = PoseStamped()
        des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z = x, y, z

        error = np.inf
        t0 = rospy.get_time()
        while error > threshold:
            if rospy.get_time()-t0 > timeout_threshold:
                print("Timeout occured")
                break

            des = np.array(
                [des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z])
            actual = np.array([self.odom.pose.pose.position.x,
                              self.odom.pose.pose.position.y, self.odom.pose.pose.position.z])
            # print("Waiting to go to pose...")
            error = np.linalg.norm(des - actual)
            time.sleep(0.1)

    def receive_trajectory(self, piece_pol):
        print("LEADER Crazyflie with id:", executor_id, "received trajectory...")
        cfid = piece_pol.cf_id

        lines = int(len(piece_pol.poly_x)/8)

        print(len(piece_pol.poly_x))

        x = np.array(piece_pol.poly_x).reshape((lines, 8))
        y = np.array(piece_pol.poly_y).reshape((lines, 8))
        z = np.array(piece_pol.poly_z).reshape((lines, 8))
        yaw = np.array(piece_pol.poly_yaw).reshape((lines, 8))
        durations = np.array(piece_pol.durations).reshape((lines, 1))

        # print("x:", x.shape)
        # print("y:", y.shape)
        # print("z:", z.shape)
        # print("yaw:", yaw.shape)
        # print("durations:", durations.shape)

        # 8 coeffs per x,y,z,yaw + 1 for duration
        matrix = np.zeros((lines, 1+8*4))
        matrix[:, 0] = durations.flatten()
        matrix[:, 1:9] = x
        matrix[:, 9:17] = y
        matrix[:, 17:25] = z
        matrix[:, 25:33] = yaw

        self.matrix = matrix

        # If both leader and follower are stabilized, start executing the trajectory
        # Otherwise, wait until both are stabilized and then do it
        if self.leader_stabilized and self.follower_stabilized and not self.traj_started:
            self.execute_trajectory_testing_leader_follower(matrix, relative=False)

    def go_to_pose(self, x, y, z, yaw, offset=[0, 0, 0]):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "world"

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

        self.go_to_pose(x, y, height, 0)

        self.wait_until_get_to_pose(
            x, y, height, 0, threshold=0.05, timeout_threshold=4)

    def take_off_traj(self, file_name=None):  # TODO:test this in experiment
        print("Taking off using trajectory...")
        # Loading file
        if file_name is None:
            file_name = "/home/marios/thesis_ws/src/crazyflie_ros/crazyflie_demo/scripts/takeoff.csv"
        tr = uav_trajectory.Trajectory().loadcsv(file_name, skip_first_row=False)
        print("duration:", tr.duration)

        # Setting ofsset because x,y are 0,0 in the trajectory file
        offset = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0]

        freq = 100  # hz
        rate = rospy.Rate(freq)
        t = 0
        dt = 1/freq
        while not rospy.is_shutdown() and t < tr.duration:
            t = t+dt

            evaluation = tr.eval(t)
            pos, yaw = evaluation.pos, evaluation.yaw
            x, y, z = pos[0], pos[1], pos[2]

            # print("Leader", "t:", t, "x:", x, "y:", y, "z:", z, "yaw:", yaw)
            self.go_to_pose(x, y, z, yaw, offset=offset)
            rate.sleep()

        self.wait_until_get_to_pose(x, y, z, yaw, threshold=0.05, timeout_threshold=3)

    def land(self):
        print("Landing...")
        # Land string is not necessary, but it is nice to have
        safety_land_publisher.publish("Land")
        follower_safety_land_publisher.publish("Land")

    def follower_land(self):
        print("Follower landing...")
        # Land string is not necessary, but it is nice to have
        follower_safety_land_publisher.publish("Land")

    def execute_trajectory_testing_leader_follower(self, matrix, relative=False):
        exec_pkg_path = rospack.get_path('execution')

        file_name = exec_pkg_path+"/resources/piecewise_pols/piecewise_pole_test_{}.csv".format(executor_id)

        np.savetxt(file_name,  matrix, delimiter=",", fmt='%.6f')
        tr = uav_trajectory.Trajectory()

        # TODO:Load trajectory without using file
        tr.loadcsv(file_name, skip_first_row=False)
        self.tr = tr
        print("duration:", tr.duration)

        if relative:
            # if trajectory relative to current position
            # the drone won't have to move before starting the trajector
            # offset = [self.odom.pose.pose.position.x,
            #           self.odom.pose.pose.position.y, self.odom.pose.pose.position.z]

            # offset = [0, 4, 1]
            offset = cf_leader_initial_pos
        else:
            # If not relative the drone has to go first at the starting position
            # offset = [0, 0, -0.5]  # -0.5 because of ceiling danger
            offset = [0, 0, 0]

        # publish executed trajectory for visualization
        publish_traj_as_path(tr, offset, path_pub)

        rospy.sleep(2)

        executor_pos.publish_sync_signal()
        start_pose = self.get_traj_start_pose()
        print("Leader going to start_pose:", start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z)

        self.go_to_pose(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z, yaw=0, offset=offset)
        self.wait_until_get_to_pose(start_pose.pose.position.x + offset[0],
                                    start_pose.pose.position.y + offset[1],
                                    start_pose.pose.position.z + offset[2], yaw=0, threshold=0.05, timeout_threshold=6)

        print("Leader is at start_pose")
        beep()

        # frequency of sending references to the controller in hz
        freq = 100
        rate = rospy.Rate(freq)  # maybe need to increase this
        t0 = rospy.get_time()

        t = 0
        dt = 1/freq   # 0.15
        dt /= 3  # time scaling factor
        self.publish_start_trajectory()  # send start signal to follower
        while not rospy.is_shutdown():
            t = t+dt
            if t > tr.duration:
                break

            evaluation = tr.eval(t)
            pos, yaw = evaluation.pos, evaluation.yaw
            x, y, z = pos[0], pos[1], pos[2]

            # print("Leader", "t:", t, "x:", x, "y:", y, "z:", z, "yaw:", yaw)
            self.go_to_pose(x, y, z, yaw, offset=offset)
            # wait until get to pose with a timeout
            # self.wait_until_get_to_pose(
            # x+offset[0], y+offset[1], z+offset[2], yaw, threshold=1, timeout_threshold=1)

            rate.sleep()

        self.land()

    def wait_to_stabilize(self, vel_thershold=0.02):
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

        self.leader_stabilized = True

    def wait_follower_to_stabilize(self):
        rate = rospy.Rate(20)
        while self.follower_stabilized_pos == None:
            check_ctrl_c()
            rate.sleep()

    def follower_stabilized_callback(self, msg: PoseStamped):
        # Follower is stabilized and then sends its position to the leader
        self.follower_stabilized_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        print("Follower stabilized at position:", self.follower_stabilized_pos)
        self.follower_stabilized = True

    def publish_sync_signal(self):
        sync_pub.publish(True)

    def publish_start_trajectory(self):
        start_traj_publisher.publish("Start")
        self.traj_started = True


def live_planning():
    # wait for odometry to be available
    executor_pos.wait_for_odometry()
    print("Waiting for odometry")
    executor_pos.wait_for_odometry()

    print("Leader waiting to stabilize...")
    executor_pos.wait_to_stabilize(vel_thershold=0.02)
    print("Leader waiting Folllower to stabilize...")
    executor_pos.wait_follower_to_stabilize()

    print("Planning with current poses as start...")
    l_pos = [executor_pos.odom.pose.pose.position.x,
             executor_pos.odom.pose.pose.position.y,
             executor_pos.odom.pose.pose.position.z]

    f_pos = executor_pos.follower_stabilized_pos
    print("Leader stabilized pos:", l_pos)
    print("Follower stabilized pos:", f_pos)
    # get rigid body start coords
    rb_x = min(l_pos[0], f_pos[0]) + abs(l_pos[0]-f_pos[0])/2
    rb_y = min(l_pos[1], f_pos[1]) + abs(l_pos[1]-f_pos[1])/2
    rb_z = min(l_pos[2], f_pos[2]) + abs(l_pos[2]-f_pos[2])/2
    rb_pos = [rb_x, rb_y, rb_z]
    rb_yaw = 0
    drones_distance = abs(rb_x)
    theta = atan2(l_pos[2]-f_pos[2], l_pos[0]-f_pos[0])
    rb_state = [rb_x, rb_y, rb_z, rb_yaw, drones_distance, theta]
    print("Rigid body start state:", rb_state)
    state = planning_state()
    state.x, state.y, state.z, state.yaw, state.drones_distance, state.drones_angle = rb_state
    start_planning_publisher.publish(state)


def planning_before_take_off():
    # wait for odometry to be available
    executor_pos.wait_for_odometry()
    print("Waiting for odometry")
    executor_pos.wait_for_odometry()

    print("Leader waiting to stabilize...")
    executor_pos.wait_to_stabilize(vel_thershold=0.02)
    print("Leader waiting Folllower to stabilize...")
    executor_pos.wait_follower_to_stabilize()

    use_already_generated_trajectories()  # TODO: remove this in real flight

    executor_pos.wait_to_receive_traj_msg()
    rospy.sleep(1)  # wait for the trajectory to be received from follower as well

    matrix = executor_pos.matrix
    executor_pos.execute_trajectory_testing_leader_follower(matrix, relative=False)
    executor_pos.land()


def use_already_generated_trajectories():
    # load trajectory file to test generated trajectories (THIS MUST BE REMOVED for real flight)
    leader_traj_file = rospy.get_param("/cf_leader_traj")
    print("LEADER: leader traj file:", leader_traj_file)
    matrix = np.loadtxt(leader_traj_file, delimiter=",", skiprows=0, usecols=range(33))
    executor_pos.matrix = matrix


def test_leader_follower():
    traj = uav_trajectory.Trajectory()

    # load trajectory file
    leader_traj_file = rospy.get_param("/cf_leader_traj")
    matrix = np.loadtxt(leader_traj_file, delimiter=",", skiprows=0, usecols=range(33))

    print(matrix.shape)
    if matrix.shape[0] == 33:
        print("Reshaping Matrix to (1,33) shape")
        matrix = matrix.reshape(1, 33)

    print("Waiting for odometry")
    executor_pos.wait_for_odometry()

    print("Leader waiting to stabilize...")
    executor_pos.wait_to_stabilize(vel_thershold=0.02)
    print("Leader waiting Folllower to stabilize...")
    executor_pos.wait_follower_to_stabilize()

    # executing trajectory
    executor_pos.execute_trajectory_testing_leader_follower(matrix, relative=False)

    executor_pos.land()


if __name__ == "__main__":
    rospy.init_node("Traj_Executor_Position_Controller", anonymous=True)
    # rospy.sleep(5)
    cf_leader_initial_pos = [rospy.get_param("/cf_leader_x"), rospy.get_param("/cf_leader_y"), rospy.get_param("/cf_leader_z")]

    print("cf_leader_initial_pos:", cf_leader_initial_pos)

    # get command line arguments
    cf_name = str(sys.argv[1])
    executor_id = get_executor_id(cf_name)

    print("Executor postion controller with id:", executor_id)

    safety_land_publisher = rospy.Publisher('safety_land', String, queue_size=10)
    follower_safety_land_publisher = rospy.Publisher('/cf_follower/safety_land', String, queue_size=10)

    pos_pub = rospy.Publisher('reference', PoseStamped, queue_size=10)

    print("Waiting to connect to reference topic..")
    while pos_pub.get_num_connections() < 1:
        check_ctrl_c()

    print("Connected to reference topic")

    executor_pos = TrajectoryExecutor_Position_Controller()
    odometry_sub = rospy.Subscriber('/pixy/vicon/{}/{}/odom'.format(cf_name, cf_name),
                                    Odometry, executor_pos.odometry_callback)

    # Subscribe to trajectory topic and then execute it after going to the first waypoint
    rospy.Subscriber('/piece_pol', TrajectoryPolynomialPieceMarios, handle_new_trajectory)

    # Publish path of the leader for visualization
    path_pub = rospy.Publisher('/cf_leader/path', Path, queue_size=10)

    # Drones communication
    start_traj_publisher = rospy.Publisher('start_trajectory', String, queue_size=10)
    follower_stab = rospy.Subscriber('/cf_follower/stabilized', PoseStamped, executor_pos.follower_stabilized_callback)

    sync_pub = rospy.Publisher('sync', Bool, queue_size=10)

    start_planning_publisher = rospy.Publisher('/start_planning', planning_state, queue_size=10)

    # waiting for the follower to be ready and connect to the leader
    while start_traj_publisher.get_num_connections() < 1:
        check_ctrl_c()

    planning_time = rospy.get_param("/planning_time")
    # test_leader_follower()
    if planning_time == "before_take_off":
        print("LEADER:Planning before take off")
        planning_before_take_off()
    elif planning_time == "live":
        print("LEADER:Planning live")
        live_planning()
    else:
        rospy.logerr("Invalid planning time:", planning_time)
        sys.exit()

    rospy.spin()
