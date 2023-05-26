#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

#include <trajectory_server_msgs/Waypoints.h>

using std::vector;

namespace ego_planner
{

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
  int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
  if( size_s <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
  auto size = static_cast<size_t>( size_s );
  std::unique_ptr<char[]> buf( new char[ size ] );
  std::snprintf( buf.get(), size, format.c_str(), args ... );
  return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

class EGOReplanFSM
{
public:
  EGOReplanFSM() {}
  ~EGOReplanFSM() {}

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE
  {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    SEQUENTIAL_START
  };
  enum TARGET_TYPE
  {
    MANUAL_TARGET = 1,
    PRESET_TARGET = 2,
    REFENCE_PATH = 3
  };
  
  std::string node_name_{"EgoPlannerFSM"};
  int drone_id_;

  /* parameters */
  geometry_msgs::Pose uav_origin_to_world_tf_; // Frame transformation from origin to world frame
  geometry_msgs::Pose world_to_uav_origin_tf_; // Frame transformation from world frame to origin

  int target_type_; // If value is 1, the goal is manually defined via a subscribed topic, else if 2, the goal is defined via pre-defined waypoints 

  // Min/max(?) distance not to replan
  double no_replan_thresh_;
  // Timeout for replanning to occur
  double replan_thresh_;
  double waypoints_[50][3];
  int form_num_;
  Eigen::MatrixXd formation_;

  // Total number of waypoints
  int waypoint_num_;
  // ID of next waypoint currently being planned to
  int wpt_id_;

  // Max planning distance for the local target
  double planning_horizen_;
  double emergency_time_;
  bool enable_fail_safe_;
  bool flag_escape_emergency_;

  // Indicates that all agents within a swarm have a provided trajectory
  bool have_recv_pre_agent_; 
  bool have_trigger_, have_target_, have_odom_, have_new_target_, touch_goal_, mandatory_stop_;
  FSM_EXEC_STATE current_state_;
  int continously_called_times_{0};

  Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
  Eigen::Vector3d end_pt_;                             // goal state
  Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state

  // List of waypoints
  std::vector<Eigen::Vector3d> wps_;
  Eigen::Vector3d formation_start_;
  Eigen::Vector3d formation_pos_;

  // Count number of FSM exec iterations
  int fsm_itr_num{0};

  /* ROS utils */
  ros::NodeHandle node_;
  // Timer to execute FSM callback
  ros::Timer exec_timer_;
  ros::Timer safety_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
  ros::Subscriber waypoints_sub_;
  ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_;

  /* planning utils */
  EGOPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;
  traj_utils::DataDisp data_disp_;

private: 

  /* state machine functions */
  void execFSMCallback(const ros::TimerEvent &e);
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

  /* safety */
  void checkCollisionCallback(const ros::TimerEvent &e);
  bool callEmergencyStop(Eigen::Vector3d stop_pos);

  /* local planning */
  bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
  bool planFromGlobalTraj(const int trial_times = 1);
  bool planFromLocalTraj(const int trial_times = 1);

  /* global trajectory */

  void waypointCallback(const geometry_msgs::PoseStampedPtr &msg); // Individual waypoint callback
  void waypointsCB(const trajectory_server_msgs::WaypointsPtr &msg);
  void planNextWaypoint(const Eigen::Vector3d next_wp, const Eigen::Vector3d previous_wp);

  /* input-output */
  void mandatoryStopCallback(const std_msgs::Empty &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
  void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

  /* ground height measurement */
  bool measureGroundHeight(double &height);

  std::pair<bool,bool> isGoalReachedAndReplanNeeded();

  // Transform the trajectory from UAV frame to world frame
  void transformMINCOTrajectoryToWorld(traj_utils::MINCOTraj & MINCO_msg){

    // Account for offset from UAV origin to world frame
    for (int i = 0; i < MINCO_msg.duration.size() - 1; i++)
    {
      MINCO_msg.inner_x[i] += uav_origin_to_world_tf_.position.x;
      MINCO_msg.inner_y[i] += uav_origin_to_world_tf_.position.y;
      MINCO_msg.inner_z[i] += uav_origin_to_world_tf_.position.z;
    }
    MINCO_msg.start_p[0] += uav_origin_to_world_tf_.position.x;
    MINCO_msg.start_p[1] += uav_origin_to_world_tf_.position.y;
    MINCO_msg.start_p[2] += uav_origin_to_world_tf_.position.z;

    MINCO_msg.end_p[0] += uav_origin_to_world_tf_.position.x;
    MINCO_msg.end_p[1] += uav_origin_to_world_tf_.position.y;
    MINCO_msg.end_p[2] += uav_origin_to_world_tf_.position.z;
  }

  // Transform the trajectory from world frame to UAV frame
  void transformMINCOTrajectoryToUAVOrigin(traj_utils::MINCOTraj & MINCO_msg){

    // Account for offset from UAV origin to world frame
    for (int i = 0; i < MINCO_msg.duration.size() - 1; i++)
    {
      MINCO_msg.inner_x[i] += world_to_uav_origin_tf_.position.x;
      MINCO_msg.inner_y[i] += world_to_uav_origin_tf_.position.y;
      MINCO_msg.inner_z[i] += world_to_uav_origin_tf_.position.z;
    }
    MINCO_msg.start_p[0] += world_to_uav_origin_tf_.position.x;
    MINCO_msg.start_p[1] += world_to_uav_origin_tf_.position.y;
    MINCO_msg.start_p[2] += world_to_uav_origin_tf_.position.z;

    MINCO_msg.end_p[0] += world_to_uav_origin_tf_.position.x;
    MINCO_msg.end_p[1] += world_to_uav_origin_tf_.position.y;
    MINCO_msg.end_p[2] += world_to_uav_origin_tf_.position.z;
  }
  
private:

  /* Logging functions */
  void logInfo(const std::string& str){
    ROS_INFO_NAMED(node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logWarn(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logError(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logFatal(const std::string& str){
    ROS_FATAL_NAMED(node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logInfoThrottled(const std::string& str, double period){
    ROS_INFO_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logWarnThrottled(const std::string& str, double period){
    ROS_WARN_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logErrorThrottled(const std::string& str, double period){
    ROS_ERROR_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

  void logFatalThrottled(const std::string& str, double period){
    ROS_FATAL_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", 
      planner_manager_->pp_.drone_id, str.c_str());
  }

};

} // namespace ego_planner

#endif