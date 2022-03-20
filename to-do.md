# To do list

**Bold** are prioritized tasks.

## Execution

- [x] Visualize obstacle and drone pose realtime while flying (test through rosbags)V
- [x] Visualize drone recorded trajectory in 3D (through rosbags)
- [x] Calculate and log euclidean distance error between reference and actual trajectory
- [ ] Fix problem of controller not correcting at the x axis
- [x] Write a script that checks if I am connected via ethernet and only then allow to fly
- [x] Add time-scaling option at executing trajectories
- [ ] ~~Dynamically change the radius threshold for the next waypoint depending on the distance~~
- [ ] Remove outliers from the trajectory comparison visualization
- [ ] Run the last generated bag file at visualizations
- [ ] **Maintain the velocity that th controller works the best and use it for the next waypoint(while genrrating)**
- [ ] **Make private repo for NMPC controller**
- [x] When leader lands land follower too
- [x] Define follower and leader path file at the formation.launch file
- [x] Visualise path at debugging.launch

## Planning

- [ ] Accept symmetrical goal states (yaw=0/180 deg)
- [ ] Design use cases that exhibit the improvements on path planning (custom mesh V shape)
- [ ] Change easily the degrees of freedom of planning (yaw,drones distance,drones angle)
- [x] **Execute trajectory for smooth take-off**
- [ ] Execute trajectory going to first waypoint of trajectory (it shouldn't be necessary )
- [x] Make a launch file and change all parameters from there (faster development)
- [x] Take into account the floor while planning (minimum distance from the floor)
- [x] **Optimize based on maximum obstacle clearance (needs test)**
- [x] Include safety distance above drones
