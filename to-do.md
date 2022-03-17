# To do list

**Bold** are prioritized tasks.

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
- [ ] **Maintain the velocity that th controller works the best and use it for the next waypoint(while genrrating)**
- [ ] **Make private repo for NMPC controller**
- [ ] When leader lands land follower too
- [ ] Define follower and leader path file at the formation.launch file
- [ ] Visualise path at debugging.launch

## Planning

- [ ] Accept symmetrical goal states (yaw=0/180 deg)
- [ ] Design use cases that exhibit the improvements on path planning (custom mesh V shape)
- [ ] Change easily the degrees of freedom of planning (yaw,drones distance,drones angle)
- [ ] **Execute trahectory for smooth take-off and going to first waypoint of trajectory**
- [ ] Make a launch file and change all parameters from there (faster development)
- [ ] Take into account the floor while planning (minimum distance from the floor)
