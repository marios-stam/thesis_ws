# To do list

**Bold** are prioritized tasks.

## Execution

- [x] Visualize obstacle and drone pose realtime while flying (test through rosbags)V
- [x] Visualize drone recorded trajectory in 3D (through rosbags)
- [x] Calculate and log euclidean distance error between reference and actual trajectory
- [ ] ~~Fix problem of controller not correcting at the x axis~~ (leave it for now)
- [x] Write a script that checks if I am connected via ethernet and only then allow to fly
- [x] Add time-scaling option at executing trajectories
- [ ] ~~Dynamically change the radius threshold for the next waypoint depending on the distance~~
- [ ] Remove outliers from the trajectory comparison visualization
- [ ] Run the last generated bag file at visualizations
- [ ] **Maintain the velocity that th controller works the best and use it for the next waypoint(while genrrating)**
- [x] **Make private repo for NMPC controller**
- [x] When leader lands land follower too
- [x] Define follower and leader path file at the formation.launch file
- [x] Visualise path at debugging.launch
- [x] Not start controller if haven't received odometry
- [x] **Leader-follower go to start points synchronised**
- [ ] **Fix synchronization error**

      [ERROR] [1648115156.367892]: bad callback: <bound method TrajectoryExecutor_Position_Controller.receive_leader_pose of <**main**.TrajectoryExecutor_Position_Controller object at 0x7f456f8d5820>>
      Traceback (most recent call last):
      File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in \_invoke_callback
      cb(msg)
      File "/home/marios/thesis_ws/src/execution/scripts/follower_traj_executor_position_controller.py", line 189, in receive_leader_pose
      t = self.traj_matcher.get_corresponding_t(
      File "/home/marios/thesis_ws/src/execution/src/leader_follower/time_based.py", line 80, in get_corresponding_t
      t = self.solve_i_pol_for_t(leader_pos, i)
      File "/home/marios/thesis_ws/src/execution/src/leader_follower/time_based.py", line 63, in solve_i_pol_for_t
      most_common_value = sorted(occurrences.items())[0][0]
      IndexError: list index out of range

- [ ] Make a script that prints the start time of the trajectories

## Planning

- [ ] Accept symmetrical goal states (yaw=0/180 deg)
- [ ] Design use cases that exhibit the improvements on path planning (custom mesh V shape)
- [ ] Change easily the degrees of freedom of planning (yaw,drones distance,drones angle)
- [x] **Execute trajectory for smooth take-off**
- [ ] ~~Execute trajectory going to first waypoint of trajectory~~ (not necessary )
- [x] Make a launch file and change all parameters from there (faster development)
- [x] Take into account the floor while planning (minimum distance from the floor)
- [x] **Optimize based on maximum obstacle clearance (needs test)**
- [x] Include safety distance above drones
