# ego_gz_bridge
This package contains the bridge to link Egoswarm V2 algorithms, the gazebo simulation environment and PX4.

# Requirements
1. Ubuntu 20.04 with ROS Noetic
2. vcstool: For repo setup 
3. Simulation
    - Gazebo Classic (Version 11)
    - PX4-Autopilot 
    - MavROS
    - Simple Quad Simulator
4. Other packages:
    - Listed in setup

# Setup
1. Install binaries
```bash 
export ROS_DISTRO="noetic"

sudo apt install tmux python3-vcstool xmlstarlet -y
# Install ROS dependencies
sudo apt install ros-noetic-tf2-sensor-msgs -y
sudo apt-get install ros-${ROS_DISTRO}-mavlink ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras -y
# sudo apt-get install ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-octomap-ros ros-${ROS_DISTRO}-control-toolbox -y
# Install external dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev libgoogle-glog-dev -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

2. Clone repositories
```bash
mkdir -p ~/raynor_ws/src/
cd ~/raynor_ws/src
git clone https://github.com/JohnTGZ/ego-px4.git
cd ego-px4/ego_gz_bridge
vcs import < thirdparty.repos --recursive
```

3. Install PX4 firmware
```bash
# cd to PX4-Autopilot repo
cd ~/raynor_ws/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx
# Make SITL target for Gazebo simulation
DONT_RUN=1 make px4_sitl_default gazebo-classic
# If you screw up, clean up the build files in the repo:
make distclean
```

# Quick start
```bash
cd ~/raynor_ws/src/ego-px4/ego_gz_bridge/scripts
# Simulation using simple quad simulator
./simple_sim.sh
# Simulation using gazebo
./gazebo_sim_single_uav.sh
```

```bash
# Taking off
rostopic pub /traj_server_event std_msgs/Int8 "data: 0" --once
# Taking Mission
rostopic pub /traj_server_event std_msgs/Int8 "data: 2" --once
```

# EGOSwarm V2 modifications

## Changes made to EGOSwarm V2
1. Added publisher in traj_server to publish PVA commands to "/mavros/setpoint_raw/local"
    - This goes to the PX4 SITL
2. Added static transform from "world" to "map"
    - This is because the ego swarm does not use frame transforms, putting every frame_id as "world"
3. To `poscmd_2_odom`:
    - Added subscription to "mavros/local_position/odom", taking that data and republishing as an "odometry" topic to be fed to ego_planner_node
4. Use Iris as simulated drone with depth camera attachment
5. Add transform from depth camera frame (`camera_link`) to iris (`base_link`)
6. Remapped `grid_map/cloud` topic in `advanced_param.xml` to `/iris_depth_camera/camera/depth/points`
7. Move model assets and  files to `ego_gz_bridge`. 
8. Changed `grid_map/frame_id` from `world` to `camera_link` in `advanced_param.xml`
9. Moved `poscmd_2_odom` and `odom_visualization` nodes from simulator.xml up one level to advanced_param.xml
10. Create another node just to transform point clouds to add publishing of point clouds from depth camera transformed from `camera_link` to `map` frame
11. Added a "complete" state machine to trajectory server with safety features such as Emergency stop and the ability to switch between HOVER and MISSION mode.
12. The occupancy map not aligned with the actual depth camera point cloud: Need to set the intrinsic parameters (This can be obtained by the 'K' variable in the `camera_info` topic )
13. Investigate being able to publish transforms via gazebo plugins
    - Method A: publish joint state topics in gazebo and have the robot_state_publisher subscribe to it and the robot description, which then publishes the TF 
14. Continue working on modifying xacro file to work on gazebo.
    - Tried to interface RotorS plugins with mavros, didn't work. Somehow I couldn't configure mavlink_simulator to accept udp connection
15. Replanning does not take into account the current position of the drone. This could be perhaps due to the issue of not being sure if the position of the drone relative to the world frame is accurate, due to possible drift from VIO.
16. Rerouted topics for gazebo simulation
17. Fixed projected poitns from depth camera not being intialized with the right size.
18. Fixed issue where drone's heading does not always face it's direction of travel (results in depth camera not facing the direction of travel and knocking into obstacles it didn't detect in it's desired trajectory). The fix is to ignore the yaw rate being supplied by the command.

## Changes TODO
### Simulation
- Test with multiple droness
- Look into gazebo plugins for quadrotor dynamics?
- Look into Promethus & px4_command

### gridmap
- Add body to camera transform as a matrix param

### Trajectory Server
- For trajectory server, read the current state of the mavros/state topic before determining the starting state machine state.
- Issue a set of waypoints via an action goal/message

### Benchmarking/Diagnostics
- Benchmark the replanning time for each drone's planner (are they close enough to the specified replanning frequency?)

### Hardware
- Test compilation on radxa

## Issues
- Disabling of offboard mode for land state would be a good feature. Current challenge to implement it is to be able to reliably check that the drone has actually landed (Otherwise it will be stuck in AUTO.LOITER while hovering in the air, being unable to disarm).

## Future TODO
- Adapt simulation to size of acutal drone to be used
- Port to ROS2
- Investigate changing the custom GridMap implementation to alternatives such as Octomap but still consider the potential performance issues with an established library.
