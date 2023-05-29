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
  /* State machine states */
  enum ServerState
  {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    SEQUENTIAL_START
  };

  /* State machine events */
  enum ServerEvent
  {
    WAIT_TARGET_E,      // 1
    GEN_NEW_TRAJ_E,     // 2
    REPLAN_TRAJ_E,      // 3
    EXEC_TRAJ_E,        // 4
    EMERGENCY_STOP_E,   // 5
    SEQUENTIAL_START_E, // 6
    EMPTY_E             // 7
  };

  /** @brief StateToString interprets the input server state **/
  const std::string StateToString(ServerState state)
  {
      switch (state)
      {
          case ServerState::INIT:   return "INIT";
          case ServerState::WAIT_TARGET:   return "WAIT_TARGET";
          case ServerState::GEN_NEW_TRAJ:   return "GEN_NEW_TRAJ";
          case ServerState::REPLAN_TRAJ: return "REPLAN_TRAJ";
          case ServerState::EXEC_TRAJ:   return "EXEC_TRAJ";
          case ServerState::EMERGENCY_STOP:   return "EMERGENCY_STOP";
          case ServerState::SEQUENTIAL_START:   return "SEQUENTIAL_START";
          default:      return "[Unknown State]";
      }
  }

  /** @brief StateToString interprets the input server event **/
  const std::string EventToString(ServerEvent event)
  {
      switch (event)
      {
          case ServerEvent::WAIT_TARGET_E:   return "WAIT_TARGET_E";
          case ServerEvent::GEN_NEW_TRAJ_E: return "GEN_NEW_TRAJ_E";
          case ServerEvent::REPLAN_TRAJ_E:   return "REPLAN_TRAJ_E";
          case ServerEvent::EXEC_TRAJ_E:   return "EXEC_TRAJ_E";
          case ServerEvent::EMERGENCY_STOP_E:   return "EMERGENCY_STOP_E";
          case ServerEvent::SEQUENTIAL_START_E:   return "SEQUENTIAL_START_E";
          case ServerEvent::EMPTY_E:   return "EMPTY_E";
          default:      return "[Unknown Event]";
      }
  }

  enum TARGET_TYPE
  {
    MANUAL_TARGET = 1,
    PRESET_TARGET = 2,
    REFENCE_PATH = 3 //TODO Remove
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
  bool have_trigger_, have_target_, have_odom_, have_new_target_, touch_goal_;
  ServerState current_state_{ServerState::INIT};
  ServerEvent server_event_{ServerEvent::EMPTY_E};
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
  ros::Timer tick_state_timer_;
  ros::Timer exec_state_timer_;
  ros::Timer safety_timer_; // TODO remove

  ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
  ros::Subscriber waypoints_sub_;
  ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_;

  /* planning utils */
  EGOPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;
  traj_utils::DataDisp data_disp_;

private: 

  /* Timer callbacks */

  /**
   * @brief Timer callback to tick State Machine
   * This callback should only ever handle transitions between states,
   * It should not attempt to call any execution function, this 
   * should be left to the execStateTimerCB callback.
   * @param e 
   */
  void tickStateTimerCB(const ros::TimerEvent &e);

  /**
   * @brief Timer callback to execute looping commands depending on the current state 
   * 
   * @param e 
   */
  void execStateTimerCB(const ros::TimerEvent &e);

  /**
   * @brief Emergency stop the drone
   * 
   * @param stop_pos 
   * @return true 
   * @return false 
   */
  bool callEmergencyStop(Eigen::Vector3d stop_pos);

  /**
   * @brief Plans a path using planner_manager_ methods
   * 
   * @param flag_use_poly_init 
   * @param flag_randomPolyTraj 
   * @return true 
   * @return false 
   */
  bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);

  /**
   * @brief Used when no local trajectory is present, calls callReboundReplan
   * 
   * @param trial_times 
   * @return true 
   * @return false 
   */
  bool planFromGlobalTraj(const int trial_times = 1);

  /**
   * @brief Used when a local trajectory has already being planned, calls callReboundReplan
   * 
   * @param trial_times 
   * @return true 
   * @return false 
   */
  bool planFromLocalTraj(const int trial_times = 1);

  /**
   * @brief Plan from current odom position to next waypoint given the direction vector of next and prev wp
   * This is done by calling planGlobalTrajWaypoints
   * And publishing this to "global_list"
   * 
   * @param next_wp 
   * @param previous_wp 
   */
  void planNextWaypoint(const Eigen::Vector3d next_wp, const Eigen::Vector3d previous_wp);

  /**
   * @brief Check if goal is reached and replanning is required
   * 
   * @return std::pair<bool,bool> 
   */
  std::pair<bool,bool> isGoalReachedAndReplanNeeded();

  /**
   * 
   */

  /**
   * @brief Perform the following checks
   * 1. Ground height
   * 2. Sensor data timeout
   * 3. From local trajectory, clearance between agent trajectories 
   * 
   * @return ServerEvent 
   */
  ServerEvent safetyChecks();

  /* Subscriber callbacks */

  /**
   * @brief Individual waypoint callback
   * 
   * @param msg 
   */
  void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);

  /**
   * @brief Multiple waypoints callback
   * 
   * @param msg 
   */
  void waypointsCB(const trajectory_server_msgs::WaypointsPtr &msg);

  void mandatoryStopCallback(const std_msgs::Empty &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
  
  /* Helper methods */
  void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);
  bool measureGroundHeight(double &height);

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
  
  /* State Machine handling methods */
  
  // TODO: Refactor this
  std::pair<int, EGOReplanFSM::ServerState> timesOfConsecutiveStateCalls();

  void printFSMExecState();

  ServerState getServerState(){
    return current_state_;
  }

  void setServerState(ServerState des_state){
    logInfo(string_format("Transitioning server state: %s -> %s", 
      StateToString(getServerState()).c_str(), StateToString(des_state).c_str()));

    if (des_state == getServerState())
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    current_state_ = des_state;
  }

  void setServerEvent(ServerEvent event)
  {
    logInfo(string_format("Set server event: %s", EventToString(event).c_str()));

    server_event_ = event;
  }

  ServerEvent getServerEvent()
  {
    // logInfo(string_format("Retrieved server event: %s", EventToString(server_event_).c_str()));
    ServerEvent event = server_event_;
    server_event_ = ServerEvent::EMPTY_E; // Reset to empty

    return event;
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