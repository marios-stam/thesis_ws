import rosbag
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from uav_trajectory import Trajectory
from geometry_msgs.msg import PoseStamped, TransformStamped

# from bagpy import bagreader
# import pandas as pd


def visualize_bag_real_trajs(file_path, ax, offset=[0, 0, 0], vis_after_start_traj=False):
    bags_path = "/home/marios/thesis_ws/bags/"
    bag = rosbag.Bag(bags_path+file_path)

    traj_topics = get_real_topics(bag)
    if vis_after_start_traj:
        t_offset = get_start_traj_time(bag)
    else:
        t_offset = bag.get_start_time()

    trajectories = generate_traj_matrix(traj_topics, bag, offset, dtype=TransformStamped, t_offset=t_offset)

    bag.close()

    trajectories = np.array(trajectories)

    visualize_trajectory_from_matrix(ax, traj_topics, trajectories)


def visualize_bag_ref_trajs(file_path, ax, offset=[0, 0, 0]):
    bags_path = "/home/marios/thesis_ws/bags/"
    bag = rosbag.Bag(bags_path+file_path)
    # bag_reader = bagreader(bags_path+file_path)

    traj_topics = get_reference_topics(bag)

    trajectories = generate_traj_matrix(
        traj_topics, bag, offset, dtype=PoseStamped)

    bag.close()

    trajectories = np.array(trajectories)

    visualize_trajectory_from_matrix(ax, traj_topics, trajectories)


def visualise_traj_from_file(file_name, skip_first_row, ax, offset=[0, 0, 0]):
    tr = Trajectory()

    tr.loadcsv(file_name, skip_first_row)
    print("duration:", tr.duration)
    t_space = np.linspace(0, tr.duration, 100)

    xs = np.zeros(len(t_space)-1)
    ys = np.zeros(len(t_space)-1)
    zs = np.zeros(len(t_space)-1)

    for i, t in enumerate(t_space[:-1]):
        evaluation = tr.eval(t)
        pos, yaw = evaluation.pos, evaluation.yaw
        x, y, z = pos[0], pos[1], pos[2]
        x, y, z = x+offset[0], y+offset[1], z+offset[2]
        # print("t:", t, "x:", x, "y:", y, "z:", z, "yaw:", yaw)

        xs[i], ys[i], zs[i] = x, y, z

    # Visualise the trajectory in 3D
    ax.scatter(xs[0], ys[0], zs[0],  c='green')       # start with green color
    ax.scatter(xs[-1], ys[-1], zs[-1], c='red')    # start with red color

    ax.plot(xs, ys, zs, 'y')

    # find min-max coordinates
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    z_min, z_max = min(zs), max(zs)

    print("x_min:", x_min, "x_max:", x_max)
    print("y_min:", y_min, "y_max:", y_max)
    print("z_min:", z_min, "z_max:", z_max)


def get_start_traj_time(bag):
    topic = '/cf_leader/start_trajectory'
    for topic, msg, t in bag.read_messages(topics=[topic]):
        return t


def generate_traj_matrix(traj_topics, bag, offset, dtype=PoseStamped, t_offset=None):
    trajectories = []
    for top in traj_topics:
        msgs_count = bag.get_message_count(top)
        trajectories .append([[0, 0, 0]]*msgs_count)

    if t_offset == None:
        t_offset = bag.get_start_time()
    else:
        t_offset = t_offset.secs

    for i in range(len(traj_topics)):
        j = 0
        for topic, msg, t in bag.read_messages(topics=[traj_topics[i]]):

            t_now = t.secs

            if t_now < t_offset:
                continue

            # id = traj_topics.index(topic)
            if dtype == PoseStamped:
                pos = msg.pose.position
                x = pos.x + offset[0]
                y = pos.y + offset[1]
                z = pos.z + offset[2]
            elif dtype == TransformStamped:
                pos = msg.transform.translation
                x = pos.x + offset[0]
                y = pos.y + offset[1]
                z = pos.z + offset[2]

            pos = [x, y, z]
            trajectories[i][j] = pos
            j += 1

        # remove all zero lines after landing
        trajectories = np.array(trajectories)
        trajectories_without_landing = []
        print("trajectories shape:", trajectories.shape)
        for i, traj in enumerate(trajectories):
            trajectories_without_landing.append(traj[~np.all(traj == 0, axis=1)])

        trajectories = trajectories_without_landing

    return trajectories


def visualize_trajectory_from_matrix(ax, traj_topics, trajectories):
    # Visualise the trajectory in 3D
    for i, traj in enumerate(trajectories):
        traj = np.array(traj)
        print(traj.shape)
        xs, ys, zs = traj[:, 0], traj[:, 1], traj[:, 2]

        # start with green color
        ax.scatter(xs[0], ys[0], zs[0],  c='green')
        ax.scatter(xs[-1], ys[-1], zs[-1], c='red')    # start with red color

        ax.plot(xs, ys, zs, label=traj_topics[i])

        # find min-max coordinates
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        z_min, z_max = min(zs), max(zs)

        print("x_min:", x_min, "x_max:", x_max)
        print("y_min:", y_min, "y_max:", y_max)
        print("z_min:", z_min, "z_max:", z_max)


def get_reference_topics(bag):
    topics = bag.get_type_and_topic_info().topics
    traj_topics = []  # topics that contain poses of the drone

    for top in topics:
        odom_index = top.find("/reference")
        if odom_index != -1:
            # print("Found!", odom_index)
            traj_topics.append(top)

        else:
            pass
    return traj_topics


def get_real_topics(bag):
    topics = bag.get_type_and_topic_info().topics
    traj_topics = []  # topics that contain poses of the drone

    for top in topics:
        odom_index = top.find("/odom")
        if odom_index != -1:
            # print("Found!", odom_index)
            traj_topics.append(top[:odom_index])

        else:
            pass
    return traj_topics


if __name__ == "__main__":

    # bag files
    bag_file_path = "03-11/auto_gen_traj_single_difficult_akash_advice5.bag"
    bag_file_path = "03-03/2022-03-03-17-33-43.bag"
    bag_file_path = "03-11/auto_gen_traj_single_difficult_kamikazi7.bag"

    # trajectories files
    auto_generated_1 = "/home/marios/thesis_ws/src/drone_path_planning/resources/trajectories/Pol_matrix_1.csv"
    auto_generated_2 = "/home/marios/thesis_ws/src/drone_path_planning/resources/trajectories/Pol_matrix_2.csv"
    fig8 = "/home/marios/thesis_ws/src/crazyflie_ros/crazyflie_demo/scripts/figure8.csv"

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    visualize_bag_real_trajs(bag_file_path, ax, offset=[0, 0, 0], vis_after_start_traj=True)
    visualize_bag_ref_trajs(bag_file_path, ax, offset=[0, 0, 0])
    # visualise_traj(auto_generated_1, True, ax, offset=[0, 0, -0.5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()
