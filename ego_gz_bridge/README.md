# ego_gz_bridge
This package contains the bridge to link Egoswarm V2 algorithms, the gazebo simulation environment and PX4.

# Requirements
1. Ubuntu 20.04 with ROS Noetic
2. vcstool: For repo setup 
3. Simulation
    - Gazebo Classic
    - PX4-Autopilot
    - MavROS
4. Swarm algorithm:
    - EgoSwarm V2
5. Other packages:
    - ros-noetic-tf2-sensor-msgs

# Setup
1. Install binaries
```bash 
export ROS_DISTRO="noetic"

sudo apt install tmux -y
sudo apt install python3-vcstool -y
sudo apt install ros-noetic-tf2-sensor-msgs -y
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

2. Clone repositories
```bash
vcs import < thirdparty.repos --recursive
```

3. Install PX4 firmware
```bash
# cd to PX4-Autopilot repo
cd ../../../PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx
# Make SITL target for Gazebo simulation
DONT_RUN=1 make px4_sitl_default gazebo-classic

# If you screw up, clean up the build files in the repo:
make distclean
```

# Quick start
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
12. Fixed transformation between camera_link and base_link

## Changes TODO
1. Test out with using just cloud topic
    - It seems to be insufficient as the gridmap also relies on the depth image and performs ray casting
2. Need to provide either camera pose or odom. Might require another node to subscribe to the transform and publish the pose 
2. Look at transform issue between base_link and map, how to broadcast that transform properly? Maybe look at XTDrone simulation?
    - Use GPS ground truth?
3. Replanning does not take into account the current position of the drone? Why is that so?
6. Extend to multiple drones

## Issues
1. Start of planned trajectory is not based on the drone's actual position but rather the drone's predicted position.
2. Drone deviates significantly from trajectory path (Perhaps because we are sending PVA commands directly to the PX4 controller, and the controller model used in the egoswarm repo might not fit the actual dynamics of the drone)
3. Drone's heading does not always face it's direction of travel (results in depth camera not facing the direction of travel)

4. Disabling of offboard mode for land state would be a good feature. Current challenge to implement it is to make sure that the drone has actually landed (Otherwise it will be stuck in AUTO.LOITER while hovering in the air, being unable to disarm).


## Future TODO
1. Need to set up proper TF Transformation
2. Create xacro file for quadcopter model with camera
3. Downsampling of point cloud