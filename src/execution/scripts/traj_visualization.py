#!/usr/bin/env python3
from itertools import count
from nav_msgs.msg import Path
import rospy


from execution.msg import TrajectoryPolynomialPieceMarios
from uav_trajectory import Trajectory
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from matplotlib import pyplot as plt

POINTS_PER_TRAJ = 20
SHOW_GRAPHS = 0


def build_matrix_from_traj_msg(piece_pol):
    lines = int(len(piece_pol.poly_x)/8)

    print(len(piece_pol.poly_x))

    x = np.array(piece_pol.poly_x).reshape((lines, 8))
    y = np.array(piece_pol.poly_y).reshape((lines, 8))
    z = np.array(piece_pol.poly_z).reshape((lines, 8))

    yaw = np.zeros((lines, 8))
    durations = np.array(piece_pol.durations).reshape((lines, 1))

    # 8 coeffs per x,y,z,yaw + 1 for duration
    matrix = np.zeros((lines, 1+8*4))
    matrix[:, 0] = durations.flatten()
    matrix[:, 1:9] = x
    matrix[:, 9:17] = y
    matrix[:, 17:25] = z
    matrix[:, 25:33] = yaw

    return matrix


class Trajectories_Handler(Marker):
    def __init__(self,  publisher: rospy.Publisher) -> None:
        self.publisher = publisher

        self.trajectories_array = Trajectory_Marker_Array()

    def update(self, index, tr: Trajectory):
        duration = tr.duration
        dt = duration/POINTS_PER_TRAJ

        positions = []
        times, vels = [], []
        xs, ys, zs = [], [], []
        vel_xs, vel_ys, vel_zs = [], [], []
        acc_xs, acc_ys, acc_zs = [], [], []

        for i in range(POINTS_PER_TRAJ):
            t = i*dt
            pos = tr.eval(t).pos

            if SHOW_GRAPHS:
                vel = tr.eval(t).vel
                acc = tr.eval(t).acc

                times.append(t)

                xs.append(pos[0])
                ys.append(pos[1])
                zs.append(pos[2])

                vel_xs.append(vel[0])
                vel_ys.append(vel[1])
                vel_zs.append(vel[2])

                acc_xs.append(acc[0])
                acc_ys.append(acc[1])
                acc_zs.append(acc[2])

            # print("t:{} --> {}".format(t, ))
            positions.append(Point(pos[0], pos[1], pos[2]))

        if len(self.trajectories_array.markers) < index+1:
            self.trajectories_array.add_trajectory(positions, "traj_"+str(len(self.trajectories_array.markers)))
        else:
            self.trajectories_array.update_curve(index, positions)

        if SHOW_GRAPHS:
            plt.clf()
            plt.subplot(3, 4, 2)
            plt.plot(times, xs)
            plt.subplot(3, 4, 6)
            plt.plot(times, ys)
            plt.subplot(3, 4, 10)
            plt.plot(times, zs)

            plt.subplot(3, 4, 3)
            plt.plot(times, vel_xs)
            plt.subplot(3, 4, 7)
            plt.plot(times, vel_ys)
            plt.subplot(3, 4, 11)
            plt.plot(times, vel_zs)

            plt.subplot(3, 4, 4)
            plt.plot(times, acc_xs)
            plt.subplot(3, 4, 8)
            plt.plot(times, acc_ys)
            plt.subplot(3, 4, 12)
            plt.plot(times, acc_zs)

            # plt.draw()
            # plt.pause(0.001)
            print("Showing graphs")
            plt.show()

    def visusalise(self):
        print("Publishing trajectories")
        self.publisher.publish(self.trajectories_array)
        # print(self.catenaries_array.markers)


class Trajectory_Marker(Marker):
    def __init__(self, positions: np.array, name: str):
        super().__init__()
        self.header.frame_id = "world"
        self.type = Marker.LINE_STRIP
        self.action = Marker.ADD

        self.header.stamp = rospy.get_rostime()
        self.ns = name
        self.id = 0

        self.lifetime = rospy.Duration(secs=1, nsecs=0)
        # marker scale
        scale = 0.005
        self.scale.x = scale
        self.scale.y = scale
        self.scale.z = scale

        # marker color
        if (name[-1] == "0"):
            self.color.a = 1.0
            self.color.r = 1.0
            self.color.g = 0.0
            self.color.b = 1.0
        else:
            self.color.a = 1.0
            self.color.r = 0.0
            self.color.g = 1.0
            self.color.b = 1.0

        # marker orientaiton
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0

        # marker position
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.0

        # marker line points
        self.points = positions

        # custom parameters
        self.start_pos = positions[0]
        self.end_pos = positions[-1]


class Trajectory_Marker_Array(MarkerArray):
    def __init__(self,):
        super().__init__()
        self.markers = []

    def add_trajectory(self, positions, name: str):
        self.markers.append(Trajectory_Marker(positions, name))

    def update_curve(self, index, positions):
        self.markers[index].points = positions


def traj_callback(piece_pol: TrajectoryPolynomialPieceMarios):
    matrix = build_matrix_from_traj_msg(piece_pol)
    tr = Trajectory()
    tr.load_from_matrix(matrix)

    # if (traj_callback.times == 0):
    handler.update(piece_pol.cf_id, tr)

    traj_callback.times += 1


traj_callback.times = 0


def path_callback(path: Path):
    # xs, ys, zs = [], [], []
    # for i in range(len(path.poses)):
    #     pos = path.poses[i].pose.position
    #     xs.append(pos.x)
    #     ys.append(pos.y)
    #     zs.append(pos.z)

    # counts = [i for i in range(len(xs))]

    # plt.clf()

    # plt.subplot(3, 4, 1)
    # plt.plot(xs)
    # plt.subplot(3, 4, 5)
    # plt.plot(ys)
    # plt.subplot(3, 4, 9)
    # plt.plot(zs)
    # print("Path len: {}".format(len(path.poses)))
    pass


if __name__ == "__main__":
    print("Starting Trajectory Visualizer")
    rospy.init_node("trajectory_viz")
    print("started")
    publisher = rospy.Publisher("trajectories_viz", MarkerArray, queue_size=10)
    handler = Trajectories_Handler(publisher)
    print("subscribing")
    rospy.Subscriber('/piece_pol', TrajectoryPolynomialPieceMarios, traj_callback)
    print("subscribed")

    rospy.Subscriber('/drone1Path', Path, path_callback)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        handler.visusalise()
        rate.sleep()
