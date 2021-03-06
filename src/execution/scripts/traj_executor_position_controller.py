#!/usr/bin/env python3

from cgi import test
from concurrent.futures import thread
import rospy
from rospy.client import INFO
import numpy as np
from geometry_msgs.msg import PoseStamped
import simpleaudio

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
    def wait_until_get_to_pose(self, x, y, z, yaw, threshold=0.4):
        # Wait until the crazyflie gets to the pose-->nomrm(error) is smaller than threshold value
        # if self.odom is None:
        #     raise Exception("No odometry received yet")

        des_pose = PoseStamped()
        des_pose.pose.position.x, des_pose.pose.position.y, des_pose.pose.position.z = x, y, z

        error = np.inf
        timeout_threshold = 4
        t0 = rospy.get_time()
        while error > threshold:
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

        self.execute_trajectory(matrix)

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
        names = ["duration",
                 "x^0", "x^1", "x^2", "x^3", "x^4", "x^5", "x^6", "x^7",
                 "y^0", "y^1", "y^2", "y^3", "y^4", "y^5", "y^6", "y^7",
                 "z^0", "z^1", "z^2", "z^3", "z^4", "z^5", "z^6", "z^7",
                 "yaw^0", "yaw^1", "yaw^2", "yaw^3", "yaw^4", "yaw^5", "yaw^6", "yaw^7"]

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
            offset = [0, 4, 1]

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

        rospy.sleep(3)

        # frequency of sending references to the controller in hz
        rate = rospy.Rate(100.0)  # maybe need to increase this
        t0 = rospy.get_time()
        # while not rospy.is_shutdown():
        #     t = rospy.get_time()-t0
        #     if t > tr.duration:
        #         break

        #     evaluation = tr.eval(t)
        #     pos, yaw = evaluation.pos, evaluation.yaw
        #     x, y, z = pos[0], pos[1], pos[2]

        #     print("t:", t, "x:", x, "y:", y, "z:", z, "yaw:", yaw)
        #     self.go_to_pose(x, y, z, yaw, offset=offset)
        #     self.wait_until_get_to_pose(
        #         x+offset[0], y+offset[1], z+offset[2], yaw, threshold=0.2)

        #     rate.sleep()
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


if __name__ == "__main__":
    rospy.init_node("Traj_Executor_Position_Controller", anonymous=True)

    # get command line arguments
    cf_name = str(sys.argv[1])
    common_prefix = "demo_crazyflie"
    # get id after prefix
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

    # Subscribe to trajectory topic and then execute it after going to the first waypoint
    # rospy.Subscriber('piece_pol', TrajectoryPolynomialPieceMarios, handle_new_trajectory)

    # test_system()  # Used to check the functionality of the system

    test_system_trajectory()

    rospy.spin()
