import collections
import sys
import os
import numpy as np
from sympy import real_roots


class trajectory_matcher_time_based:
    def __init__(self, matrix):
        # matrix is the matrix of the trajectory of the leader
        self.durations = matrix[:, 0]
        self.xs = matrix[:, 1:9]
        self.ys = matrix[:, 9:17]
        self.zs = matrix[:, 17:25]
        self.yaws = matrix[:, 25:33][::-1]

        self.prev_index = 0

    def solve_i_pol_for_t(self, leader_pos, i):
        # finds t of the polynomials of the ith row of the matrix
        durations = self.durations
        tmin = 0
        tmax = durations[i]
        # get first line

        try:
            txs = trajectory_matcher_time_based.solve_for_y(
                self.xs[i, :][::-1], leader_pos[0])
            tys = trajectory_matcher_time_based.solve_for_y(
                self.ys[i, :][::-1], leader_pos[1])
            tzs = trajectory_matcher_time_based.solve_for_y(
                self.zs[i, :][::-1], leader_pos[2])
        except:
            txs = trajectory_matcher_time_based.solve_for_y(
                self.xs[i, :][::-1], leader_pos.x)
            tys = trajectory_matcher_time_based.solve_for_y(
                self.ys[i, :][::-1], leader_pos.y)
            tzs = trajectory_matcher_time_based.solve_for_y(
                self.zs[i, :][::-1], leader_pos.z)

        tx = trajectory_matcher_time_based.keep_valid_t(txs, tmin, tmax)
        ty = trajectory_matcher_time_based.keep_valid_t(tys, tmin, tmax)
        tz = trajectory_matcher_time_based.keep_valid_t(tzs, tmin, tmax)

        # print("tx:", tx, "ty:", ty, "tz:", tz)

        non_solvable_t = 0
        for i in [tx, ty, tz]:
            if len(i) == 0:
                non_solvable_t = non_solvable_t + 1

        if non_solvable_t > 1:
            return None

        ts = np.array([])
        np.concatenate
        ts = np.concatenate((ts, tx))
        ts = np.concatenate((ts, tx))
        ts = np.concatenate((ts, tx))
        occurrences = collections.Counter(ts)
        # print("occurrences:", occurrences)
        # get most common value
        most_common_value = sorted(occurrences.items())[0][0]

        # print("most common value:",
        # most_common_value)
        return most_common_value

    def get_corresponding_t(self, leader_pos):
        # leader pos is a list of [x,y,z]
        durations = self.durations
        i = self.prev_index
        t = None
        while t == None:
            # print("=========================={}==========================".format(i))
            # print("t_start:", sum(durations[: i]), end=" ")
            # print("t_end:", sum(durations[: i+1]))
            # print("t_max", durations[i])

            t = self.solve_i_pol_for_t(leader_pos, i)
            # print("t:", t)
            i = i+1
            # input()

        self.prev_index = i-1
        t = sum(self.durations[: self.prev_index])+t
        # print("t:", t)

        return t

    def keep_valid_t(ts: np.array, tmin, tmax):
        # ts : list of times
        ts = ts.real
        return np.array([t for t in ts if tmin <= t <= tmax])

    def solve_for_y(poly_coeffs, y):
        pc = poly_coeffs.copy()
        pc[-1] -= y
        roots = np.roots(pc)
        real_roots = []
        for r in roots:
            if type(r) != np.complex128 or r.imag == 0:
                real_roots.append(r)

        return np.array(real_roots)


def test():
    # load trajectory file
    traj_file_name = "/home/marios/thesis_ws/src/crazyflie_ros/crazyflie_demo/scripts/figure8.csv"

    matrix = np.loadtxt(traj_file_name, delimiter=",",
                        skiprows=1, usecols=range(33))

    matcher = trajectory_matcher_time_based(matrix)
    # t: 0.07356752525252525 x: 2.3671222828339965e-05 y: -3.8149882711119446e-05 z: 0.0 yaw: 0.0
    leader_pos = [2.3671222828339965e-05, -3.8149882711119446e-05, 0.0]
    matcher.get_corresponding_t(leader_pos)

    # t: 1.839188131313131 x: 0.9508312059227128 y: -0.21242448466646902 z: 0.0 yaw: 0.0
    leader_pos = [0.9508312059227128, -0.21242448466646902, 0.0]
    matcher.get_corresponding_t(leader_pos)

    # t: 1.9863231818181817 x: 0.9830702810065 y: -0.06176073650510305 z: 0.0 yaw: 0.0
    leader_pos = [0.9830702810065, -0.06176073650510305, 0.0]
    matcher.get_corresponding_t(leader_pos)


if __name__ == "__main__":
    test()
