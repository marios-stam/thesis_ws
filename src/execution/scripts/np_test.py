from scipy import stats
import sys
import os
import numpy as np
# # load trajectory file
traj_file_name = "/home/marios/thesis_ws/src/crazyflie_ros/crazyflie_demo/scripts/figure8_without_1st_line.csv"
matrix = np.loadtxt(traj_file_name, delimiter=",",
                    skiprows=1, usecols=range(33))


# for index, line in enumerate(open(traj_file_name, 'r').readlines()):
#     w = line.split(' ')
#     l1 = w[1:8]
#     l2 = w[8:15]

#     try:
#         list1 = map(float, l1)
#         list2 = map(float, l2)
#     except ValueError:
#         print('Line {i} is corrupt!'.format(i=index))
#         break
