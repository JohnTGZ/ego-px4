#include <ros/ros.h>

#include <optimizer/poly_traj_utils.hpp>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <traj_utils/PolyTraj.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>

using namespace Eigen;

/* State machine  */
enum ServerState
{
  INIT,
  IDLE,
  TAKEOFF,
  LAND,
  HOVER,
  MISSION,
  E_STOP,
};

/* State machine  */
enum ServerEvent
{
  TAKEOFF_E,        // 0
  LAND_E,           // 1
  MISSION_E,        // 2
  CANCEL_MISSION_E, // 3
  E_STOP_E,         // 4
  EMPTY_E,          // 5
};

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

class TrajServer{
public:

  void init(ros::NodeHandle& nh);

  /* ROS Callbacks */

  /**
   * Callback for heartbeat from Ego Planner
  */
  void heartbeatCallback(std_msgs::EmptyPtr msg);

  /**
   * Callback for polynomial trajectories
  */
  void polyTrajCallback(traj_utils::PolyTrajPtr msg);

  void UAVStateCb(const mavros_msgs::State::ConstPtr &msg);

  void UAVPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg);

  void serverEventCb(const std_msgs::Int8::ConstPtr & msg);

  /**
   * Timer callback to generate PVA commands for executing trajectory.
   * It will determine what sort of trajectory execution (takeoff, landing, hover, mission etc.)
   * based on the current state of the state machine.
  */
  void execTrajTimerCb(const ros::TimerEvent &e);

  /**
   * Timer callback to tick State Machine
   * This callback should only ever handle transitions between states,
   * It should not attempt to call any trajectory execution function, this 
   * should be left to the execTrajTimerCb callback.
  */
  void tickServerStateTimerCb(const ros::TimerEvent &e);

  /* Trajectory execution methods */

  /**
   * Execute take off 
  */
  void execTakeOff();

  /**
   * Execute hover
  */
  void execHover();

  /**
   * Execute landing
  */
  void execLand();

  /**
   * Execute Mission Trajectory
  */
  void execMission();

  /**
   * Start a mission
  */
  void startMission();

  /**
   * End the current mission.
   * For all purposes and intent, ending the mission is same as cancelling it.
  */
  void endMission();

  /* Conditional checking methods */

  /**
   * Check if landing execution is complete
  */
  bool isLanded();

  /**
   * Check if take off execution is complete
  */
  bool isTakenOff();

  /**
   * Check if mission execution is complete.
  */
  bool isMissionComplete();

  /**
   * Check if heartbeat from planner has timed out
  */
  bool isPlannerHeartbeatTimeout();

  /* Helper methods */

  /**
   * Publish PVA (Position, Velocity, Acceleration) commands
  */
  void publish_cmd(
    Vector3d p, Vector3d v, Vector3d a, 
    Vector3d j, double yaw, double yaw_rate, 
    uint16_t type_mask = 0);

  std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt);

  /**
   * Send request for PX4 to switch to offboard mode and arm
  */
  bool toggle_offboard_mode(bool toggle);

  /**
   * Checks if UAV state is:
   * 1. In OFFBOARD mode 
   * 2. ARMED
  */
  bool is_uav_ready() {
    return (uav_current_state_.mode == "OFFBOARD") 
      && uav_current_state_.armed;
  }

  /**
   * Checks if UAV state is:
   * 1. In "AUTO.LOITER" mode 
   * 2. DISARMED
  */
  bool is_uav_idle() {
    return (uav_current_state_.mode == "AUTO.LOITER") 
      && !uav_current_state_.armed;
  }

  /** @brief StateToString interprets the input server state **/
  const std::string StateToString(ServerState state)
  {
      switch (state)
      {
          case ServerState::INIT:   return "INIT";
          case ServerState::IDLE:   return "IDLE";
          case ServerState::TAKEOFF:   return "TAKEOFF";
          case ServerState::LAND: return "LAND";
          case ServerState::HOVER:   return "HOVER";
          case ServerState::MISSION:   return "MISSION";
          case ServerState::E_STOP:   return "E_STOP";
          default:      return "[Unknown State]";
      }
  }

  /** @brief StateToString interprets the input server state **/
  const std::string EventToString(ServerEvent state)
  {
      switch (state)
      {
          case ServerEvent::TAKEOFF_E:   return "TAKEOFF";
          case ServerEvent::LAND_E: return "LAND";
          case ServerEvent::MISSION_E:   return "MISSION";
          case ServerEvent::EMPTY_E:   return "EMPTY";
          case ServerEvent::E_STOP_E:   return "E_STOP";
          default:      return "[Unknown Event]";
      }
  }

  /* Send a server event to be processed by the state machine*/
  void setServerEvent(ServerEvent event);

  /* Retrieve a server event to be processed by the state machine,
  it will then set the event to be EMPTY, which prevents further processing*/
  ServerEvent getServerEvent();

  /** Transition state machine to new_state.
   * This should ONLY be called within tickServerStateTimerCb.
   */
  void setServerState(ServerState new_state); 

  /** get current server state */
  ServerState getServerState(); 

private:
  int drone_id_; // ID of drone being commanded by trajectory server instance
  std::string origin_frame_; // frame that the drone originated from i.e. it's local pose is (0,0,0) w.r.t to this frame.

  /* Publishers, Subscribers and Timers */
  ros::Publisher pos_cmd_raw_pub_; // Publisher of commands for PX4 
  ros::Publisher pos_cmd_pub_; // Publisher of commands for PX4 
  
  ros::Subscriber poly_traj_sub_; // Subscriber for polynomial trajectory
  ros::Subscriber heartbeat_sub_; // Subscriber for heartbeat
  ros::Subscriber uav_state_sub_; // Subscriber for UAV State (MavROS)
  ros::Subscriber pose_sub_; // Subscriber for UAV State (MavROS)
  // TODO: make this a service server
  ros::Subscriber set_server_state_sub_; // Subscriber for setting server state

  ros::Timer exec_traj_timer_; // Timer to generate PVA commands for trajectory execution
  ros::Timer tick_sm_timer_; // Timer to tick the state machine 

  /** @brief Service clients **/
  ros::ServiceClient arming_client, set_mode_client; 

  /* Stored data*/
  ServerEvent server_event_{ServerEvent::EMPTY_E};
  ServerState server_state_{ServerState::INIT};
  mavros_msgs::State uav_current_state_;

  geometry_msgs::PoseStamped uav_pose_;
  boost::shared_ptr<poly_traj::Trajectory> traj_;
  Eigen::Vector3d last_mission_pos_;

  // yaw control
  double last_mission_yaw_{0.0}, last_mission_yaw_dot_{0.0};

  // Flags
  double traj_duration_;
  ros::Time start_time_;
  ros::Time heartbeat_time_{0};

  bool mission_completed_{true};
  // bool heartbeat_timeout_{true};

  // Params
  std::string node_name_{"traj_server"};
  double time_forward_; // Used to calculate yaw 
  double pub_cmd_freq_; // Frequency to publish PVA commands
  double sm_tick_freq_; // Frequency to tick the state machine transitions

  double planner_heartbeat_timeout_{0.5}; // Planner heartbeat timeout

  double takeoff_height_{0.0}; // Default height to take off to
  double landed_height_{0.05}; // We assume that the ground is even (z = 0)
  double take_off_landing_tol_{0.05};


private:
  void logInfo(const std::string& str){
    ROS_INFO_NAMED(node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logWarn(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logError(const std::string& str){
    ROS_WARN_NAMED(node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logFatal(const std::string& str){
    ROS_FATAL_NAMED(node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logInfoThrottled(const std::string& str, double period){
    ROS_INFO_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logWarnThrottled(const std::string& str, double period){
    ROS_WARN_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logErrorThrottled(const std::string& str, double period){
    ROS_ERROR_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

  void logFatalThrottled(const std::string& str, double period){
    ROS_FATAL_THROTTLE_NAMED(period, node_name_, "UAV_%i: %s", drone_id_, str.c_str());
  }

};

