# To do list

## Execution

- [x] Visualize obstacle and drone pose realtime while flying (test through rosbags)V
- [x] Visualize drone recorded trajectory in 3D (through rosbags)
- [x] Calculate and log euclidean distance error between reference and actual trajectory
- [ ] Fix problem of controller not correcting at the x axis
- [x] Write a script that checks if I am connected via ethernet and only then allow to fly
- [x] Add time-scaling option at executing trajectories
- [ ] Dynamically change the radius threshold for the next waypoint depending on the distance to the next waypoint
- [ ] Remove outliers from the trajectory comparison visualization
- [ ] Run the last generated bag file at visualizations

## Planning

- [ ] Accept symmetrical goal states (yaw=0/180 deg)
- [ ] Design use cases that exhibit the improvemens on path planning (custom mesh V shape)
- [ ] Change easily the degrees of freedom of planning (yaw,drones distance,drones angle)
