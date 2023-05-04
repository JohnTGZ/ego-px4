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

using namespace Eigen;

/* State machine  */
enum ServerState
{
  INIT,
  IDLE,
  TAKEOFF,
  LAND,
  HOVER,
  MISSION
};

/* State machine  */
enum ServerEvent
{
  TAKEOFF,
  LAND,
  READY_FOR_MISSION,
  EMPTY,
};


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
   * Timer callback to generate PVA commands for executing trajectory
  */
  void execTrajTimerCb(const ros::TimerEvent &e);

  /**
   * Timer callback to tick State Machine
  */
  void tickSMTimerCb(const ros::TimerEvent &e);

  /* Helper methods */

  /**
   * Publish PVA (Position, Velocity, Acceleration) commands
  */
  void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double yaw, double yaw_rate);

  std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt);

  /* Trajectory execution methods */

  /**
   * Take off 
  */
  void execTakeOff();

  /**
   * Hover
  */
  void execHover();

  /**
   * Landing
  */
  void execLand();

  /**
   * Execute Mission Trajectory
  */
  void execMissionTraj();

  /**
   * Send request for PX4 to switch to offboard mode and arm
  */
  bool enable_offboard_mode()
  {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Make sure takeoff is not immediately sent, 
    // this will help to stream the correct data to the program first.
    // Will give a 1sec buffer
    // ros::Duration(1.0).sleep();

    ros::Rate rate(1/pub_cmd_freq_);

    // send a few setpoints before starting
    for (int i = 10; ros::ok() && i > 0; --i)
    {
      publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
      ros::spinOnce();
      rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    bool is_offboard_and_armed = (uav_current_state_.mode == "OFFBOARD") && uav_current_state_.armed;
    ros::Time last_request_t = ros::Time::now();

    while (!is_offboard_and_armed){
      bool request_timeout = (ros::Time::now() - last_request_t > ros::Duration(2.0));

      if (uav_current_state_.mode != "OFFBOARD" && request_timeout)
      {
        ROS_INFO_NAMED(node_name_, "Setting offboard mode");
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO_NAMED(node_name_, "Successfully set offboard mode");
        }

        last_request_t = ros::Time::now();
      }
      else if (!uav_current_state_.armed && request_timeout) 
      {
        ROS_INFO_NAMED(node_name_, "Arming...");

        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO_NAMED(node_name_, "Successfully armed.");
        }

        last_request_t = ros::Time::now();
      }

      is_offboard_and_armed = (uav_current_state_.mode == "OFFBOARD") && uav_current_state_.armed;

      ros::spinOnce();
      rate.sleep();        
    }

  }

  /**
   * Checks if UAV state is:
   * 1. Connected
   * 2. In OFFBOARD mode 
   * 3. ARMED
  */
  bool is_uav_ready() {
    return (uav_current_state_.mode == "OFFBOARD") 
      && uav_current_state_.armed 
      && uav_current_state_.connected;
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
          default:      return "[Unknown State]";
      }
  }

  /** @brief StateToString interprets the input server state **/
  const std::string EventToString(ServerEvent state)
  {
      switch (state)
      {
          case ServerEvent::TAKEOFF:   return "TAKEOFF";
          case ServerEvent::LAND: return "LAND";
          case ServerEvent::READY_FOR_MISSION:   return "READY_FOR_MISSION";
          case ServerEvent::EMPTY:   return "EMPTY";
          default:      return "[Unknown Event]";
      }
  }

  /* Send a server event to be processed by the state machine*/
  void setEvent(ServerEvent event);

  /* Retrieve a server event to be processed by the state machine,
  it will then set the event to be EMPTY, which prevents further processing*/
  ServerEvent getEvent();

  /** Transition state machine to new_state */
  void setServerState(ServerState new_state); 

  /** Transition state machine to new_state */
  ServerState getServerState(); 

private:

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
  ServerEvent server_event_{ServerEvent::EMPTY};
  ServerState server_state_{ServerState::INIT};
  mavros_msgs::State uav_current_state_;

  geometry_msgs::PoseStamped uav_pose_;
  boost::shared_ptr<poly_traj::Trajectory> traj_;
  Eigen::Vector3d last_pos_;

  // yaw control
  double last_yaw_{0.0}, last_yawdot_{0.0};

  // Flags
  bool receive_traj_ = false;
  double traj_duration_;
  ros::Time start_time_;
  int traj_id_;
  ros::Time heartbeat_time_{0};

  // Params
  std::string node_name_{"traj_server"};
  double time_forward_; // Used to calculate yaw 
  double pub_cmd_freq_; // Frequency to publish PVA commands
  double sm_tick_freq_; // Frequency to tick the state machine transitions

  double takeoff_height_; // Default height to take off to

};
