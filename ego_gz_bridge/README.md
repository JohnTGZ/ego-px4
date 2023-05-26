# ego_gz_bridge
This package contains the bridge to link Egoswarm V2 algorithms, the gazebo simulation environment and PX4.

# Requirements
1. Operating System/Frameworks
    - Ubuntu 20.04 
    - ROS Noetic
2. Tools
    - vcstool: For repo setup 
    - xmlstarlet: For multi-vehicle simulation
3. Simulation
    - Gazebo Classic (Version 11)
    - PX4-Autopilot
    - Simple Quad Simulator
4. Other packages:
    - MavROS 
    - Listed in "Setup" section

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
cp -r ~/raynor_ws/src/ego-px4/ego_gz_bridge/simulation/models/raynor ~/raynor_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/

# If you screw up, clean up the build files in the repo:
make distclean
```

# Quick start
```bash
cd ~/raynor_ws/src/ego-px4/ego_gz_bridge/scripts
# Simulation using simple quad simulator
./simple_sim.sh
# Simulation using gazebo
./gazebo_sim_multi_uav.sh
```

```bash
# Taking off
rostopic pub /traj_server_event std_msgs/Int8 "data: 0" --once
# Taking Mission
rostopic pub /traj_server_event std_msgs/Int8 "data: 2" --once
```
# Changes made to EGOSwarm V2
1. Added a "complete" state machine to trajectory server with safety features such as Emergency stop and the ability to switch between HOVER and MISSION mode.
2. The occupancy map not aligned with the actual depth camera point cloud: Need to set the intrinsic parameters (This can be obtained by the 'K' variable in the `camera_info` topic )
4. Fixed projected points from depth camera not being intialized with the right size.
5. Fixed issue where drone's heading does not always face it's direction of travel (results in depth camera not facing the direction of travel and knocking into obstacles it didn't detect in it's desired trajectory). The fix is to ignore the yaw rate being supplied by the command.
6. Got at least 2 drones to plan using ego planner in gazebo.
7. Set the physical form of the drone properly 
    - Motor to motor: 0.12m
    - Rotor diameter: 0.09m
    - Size can be approximated as a cuboid: 0.21 * 0.21 * 0.12
8. Offset the starting mavros local position of the drone by creating new frames and using those as the origin frame for each drone
9. Fixed issues with simulating multiple robots in Gazebo Classic
10. Add a visualization mesh to each drone (Similar to simple quad simulator's implementation)
11. Use a new mesh (fake_drone.dae) model to represent the drone in gazebo
12. Create script to takeoff and switch to mission mode.

# Demo
1. PX4 State control
    - Demo takeoff, landing
2. Multiple drones in Gazebo
    - Demo with 2 drones
    - Demo with 4 drones

# Changes TODO
## Simulation
- In Trajectory server
    - Be able to issue a set of waypoints via an action goal/message
        - Cancel/Start/Pause execution
        - Specify formations to execute waypoints
        - Check if every UAV in formation has finished execution of current waypoint before planning for the next one
        - Trajectory Server should trigger planner to start planning (Via a service call)
    - Add script execute a set of waypoints, then land.
- Extend to 5 drones
    - Seemingly, Issue with running 5 drones in gazebo is that it detects the drones and treats it as an obstacle: resulting in planning collision
        - One solution would be to use the drone_detection module to remove the drone point cloud, assuming it works in simulation
- Look into gazebo plugins for quadrotor dynamics?
    - Promethus & px4_command and SE03 Simulator (within egoswarm v2 repo)
- Set up a more complex simulation world
- In Gazebo, we need a new mesh to visualize the robot (current one is simply scaled down from iris, and looks weird, perhaps we could use hummingbird model?)

## gridmap
- Add body to camera transform as a matrix ROS Param (Make sure that it is same as that in simulation)
- Take in intrinsic params of camera via camera_info topic

## Trajectory Server
- For trajectory server, read the current state of the mavros/state topic before determining the starting state machine state.
- Disabling of offboard mode for land state would be a good feature. Current challenge to implement it is to be able to reliably check that the drone has actually landed (Otherwise it will be stuck in AUTO.LOITER while hovering in the air, being unable to disarm).

## Benchmarking/Diagnostics
- Benchmark the replanning time for each drone's planner (are they close enough to the specified replanning frequency?)

## Hardware
- Test compilation on radxa

# Issues
- Replanning does not take into account the current position of the drone. This could be perhaps due to the issue of not being sure if the position of the drone relative to the world frame is accurate, due to possible drift from VIO.

# Future Roadmap
- Solidify framework for managing multiple robots
- Port to ROS2
- Investigate changing the custom GridMap implementation to alternatives such as Octomap but still consider the potential performance issues with an established library.
