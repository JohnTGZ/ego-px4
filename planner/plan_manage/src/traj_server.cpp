#include <plan_manage/traj_server.h>

using namespace Eigen;

void TrajServer::init(ros::NodeHandle& nh)
{
  ROS_INFO_NAMED(node_name_, "Initializing");

  /* Params*/
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/pub_cmd_freq", pub_cmd_freq_, 25.0);
  nh.param("traj_server/state_machine_tick_freq", sm_tick_freq_, 50.0);
  nh.param("traj_server/takeoff_height", takeoff_height_, 1.0);

  /* Subscribers */
  poly_traj_sub_ = nh.subscribe("planning/trajectory", 10, &TrajServer::polyTrajCallback, this);
  heartbeat_sub_ = nh.subscribe("heartbeat", 10, &TrajServer::heartbeatCallback, this);
  uav_state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &TrajServer::UAVStateCb, this);

  pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &TrajServer::UAVPoseCB, this);

  set_server_state_sub_ = nh.subscribe<std_msgs::Int8>("/traj_server_event", 10, &TrajServer::serverEventCb, this);

  /* Publishers */
  pos_cmd_raw_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 50);

  /* Service clients */
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  /* Timer callbacks */
  exec_traj_timer_ = nh.createTimer(ros::Duration(1/pub_cmd_freq_), &TrajServer::execTrajTimerCb, this);
  tick_sm_timer_ = nh.createTimer(ros::Duration(1/sm_tick_freq_), &TrajServer::tickServerStateTimerCb, this);
}

/* ROS Callbacks */

/* Subscriber Callbacks */

void TrajServer::heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void TrajServer::polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  // The chunk of code below is just converting the received 
  // trajectories into poly_traj::Trajectory type and storing it

  // piece_nums is the number of Pieces in the trajectory 
  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  // traj_id_ = msg->traj_id;

  startMission();
}

void TrajServer::UAVStateCb(const mavros_msgs::State::ConstPtr &msg)
{
  // ROS_INFO_THROTTLE_NAMED(5.0, node_name_, "State: Mode[%s], Connected[%d], Armed[%d]", msg->mode.c_str(), msg->connected, msg->armed);
  uav_current_state_ = *msg;
}

void TrajServer::UAVPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  uav_pose_ = *msg;
}

void TrajServer::serverEventCb(const std_msgs::Int8::ConstPtr & msg)
{
  if (msg->data < 0 || msg->data > ServerEvent::EMPTY_E){
    ROS_ERROR_NAMED(node_name_, "Invalid server event, ignoring...");
  } 
  setServerEvent(ServerEvent(msg->data));
}

/* Timer Callbacks */

void TrajServer::execTrajTimerCb(const ros::TimerEvent &e)
{
  // ROS_INFO_THROTTLE_NAMED(1.0, node_name_, "Exec Traj timer cb");
  switch (getServerState()){
    case ServerState::INIT:
      // Do nothing, drone is not initialized
      break;
    
    case ServerState::IDLE:
      // Do nothing, drone has not taken off
      break;
    
    case ServerState::TAKEOFF:
      execTakeOff();
      break;
    
    case ServerState::LAND:
      execLand();
      break;
    
    case ServerState::HOVER:
      execHover();
      break;
    
    case ServerState::MISSION:
      if (isMissionComplete()){
        ROS_INFO_THROTTLE_NAMED(2.0, node_name_, "Waiting for mission...");
        execHover();
      }
      else {
        if (isPlannerHeartbeatTimeout()){
          ROS_ERROR("[traj_server] Lost heartbeat from the planner, is he dead?");
          execHover();
        }

        execMission();
      }
      break;

    case ServerState::E_STOP:
      // Do nothing, drone should stop all motors immediately
      break;
  }

}

void TrajServer::tickServerStateTimerCb(const ros::TimerEvent &e)
{
  // ROS_INFO_THROTTLE_NAMED(5.0, node_name_, "Current Server State: [%s]", StateToString(getServerState()).c_str());

  switch (getServerState())
  {
    case ServerState::INIT:
      {
        // Wait for FCU Connection
        if (uav_current_state_.connected){
          setServerState(ServerState::IDLE);
        }
        else {
          ROS_INFO_THROTTLE_NAMED(1.0, node_name_, "[INIT] Initializing Server, waiting for connection to FCU...");
        }

        break;
      }
    case ServerState::IDLE:
      ROS_INFO_THROTTLE_NAMED(1.0, node_name_, "[IDLE] Ready to take off");

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          ROS_INFO_NAMED(node_name_, "[IDLE] UAV Attempting takeoff");
          setServerState(ServerState::TAKEOFF);
          break;
        case LAND_E:
          ROS_WARN_NAMED(node_name_, "[IDLE] IGNORED EVENT. UAV has not taken off, unable to LAND");
          break;
        case MISSION_E:
          ROS_WARN_NAMED(node_name_, "[IDLE] IGNORED EVENT. Please TAKEOFF first before setting MISSION mode");
          break;
        case CANCEL_MISSION_E:
          ROS_WARN_NAMED(node_name_, "[IDLE] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          ROS_FATAL_NAMED(node_name_, "[IDLE] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }
      break;
    
    case ServerState::TAKEOFF:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          ROS_WARN_NAMED(node_name_, "[TAKEOFF] IGNORED EVENT. UAV already attempting taking off");
          break;
        case LAND_E:
          ROS_INFO_NAMED(node_name_, "[TAKEOFF] Attempting landing");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          ROS_WARN_NAMED(node_name_, "[TAKEOFF] IGNORED EVENT. Wait until UAV needs to take off before accepting mission command");
          break;
        case CANCEL_MISSION_E:
          ROS_WARN_NAMED(node_name_, "[TAKEOFF] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          ROS_FATAL_NAMED(node_name_, "[TAKEOFF] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (!is_uav_ready()){
        ROS_INFO("Calling toggle offboard mode");
        toggle_offboard_mode(true);
      }

      if (isTakenOff()){
        ROS_INFO_NAMED(node_name_, "[TAKEOFF] Take off complete");
        setServerState(ServerState::HOVER);
      }
      else {
        ROS_INFO_THROTTLE_NAMED(1.0, node_name_, "[TAKEOFF] Taking off...");
      }

      break;
    
    case ServerState::LAND:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          ROS_INFO_NAMED(node_name_, "[LAND] UAV Attempting takeoff");
          setServerState(ServerState::TAKEOFF);
          break;
        case LAND_E:
          ROS_WARN_NAMED(node_name_, "[LAND] IGNORED EVENT. UAV already attempting landing");
          break;
        case MISSION_E:
          ROS_WARN_NAMED(node_name_, "[LAND] IGNORED EVENT. UAV is landing, it needs to take off before accepting mission command");
          break;
        case CANCEL_MISSION_E:
          ROS_WARN_NAMED(node_name_, "[LAND] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          ROS_FATAL_NAMED(node_name_, "[LAND] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (isLanded()){
        ROS_INFO_NAMED(node_name_, "[LAND] Landing complete");
        setServerState(ServerState::IDLE);
      }
      else {
        ROS_INFO_THROTTLE_NAMED(1.0, node_name_, "[LAND] landing...");
      }

      setServerState(ServerState::IDLE);
      break;
    
    case ServerState::HOVER:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          ROS_WARN_NAMED(node_name_, "[HOVER] IGNORED EVENT. UAV already took off. Currently in [HOVER] mode");
          break;
        case LAND_E:
          ROS_INFO_NAMED(node_name_, "[HOVER] Attempting landing");
          setServerState(ServerState::LAND);
          break;
        case MISSION_E:
          ROS_INFO_NAMED(node_name_, "[HOVER] UAV entering [MISSION] mode.");
          setServerState(ServerState::MISSION);
          break;
        case CANCEL_MISSION_E:
          ROS_WARN_NAMED(node_name_, "[HOVER] IGNORED EVENT. No mission to cancel");
          break;
        case E_STOP_E:
          ROS_FATAL_NAMED(node_name_, "[HOVER] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      break;
    
    case ServerState::MISSION:

      switch (getServerEvent())
      {
        case TAKEOFF_E:
          ROS_WARN_NAMED(node_name_, "[MISSION] IGNORED EVENT. UAV already took off. Currently in [MISSION] mode");
          break;
        case LAND_E:
          ROS_WARN_NAMED(node_name_, "[MISSION] IGNORED EVENT. Please cancel mission first");
          break;
        case MISSION_E:
          ROS_WARN_NAMED(node_name_, "[MISSION] IGNORED EVENT. UAV already in [MISSION] mode");
          break;
        case CANCEL_MISSION_E:
          ROS_WARN_NAMED(node_name_, "[MISSION] Mission cancelled!");
          endMission();
          setServerState(ServerState::HOVER);
          break;
        case E_STOP_E:
          ROS_FATAL_NAMED(node_name_, "[MISSION] EMERGENCY STOP ACTIVATED!");
          setServerState(ServerState::E_STOP);
          break;
        case EMPTY_E:
          // Default case if no event sent
          break;
      }

      if (!isMissionComplete()){
        ROS_INFO_THROTTLE_NAMED(1.0, node_name_, "[MISSION] executing mission...");
      }
      break;

    case ServerState::E_STOP:
      ROS_FATAL_THROTTLE_NAMED(0.5, node_name_, "[E_STOP] Currently in E STOP State, please reset the vehicle and trajectory server!");
      break;

  }
}

/* Trajectory execution methods */

void TrajServer::execLand()
{
  Eigen::Vector3d pos;
  pos << uav_pose_.pose.position.x, uav_pose_.pose.position.y, landed_height_;
  uint16_t type_mask = 2552; // Ignore Velocity, Acceleration

  publish_cmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0, type_mask);
}

void TrajServer::execTakeOff()
{ 
  Eigen::Vector3d pos;
  pos << uav_pose_.pose.position.x, uav_pose_.pose.position.y, takeoff_height_;
  uint16_t type_mask = 2552; // Ignore Velocity, Acceleration

  publish_cmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0, type_mask);

}

void TrajServer::execHover()
{
  uint16_t type_mask = 2552; // Ignore Velocity, Acceleration

  publish_cmd(last_mission_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0, type_mask);
}

void TrajServer::execMission()
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }

  // Moved up one level
  // if (isPlannerHeartbeatTimeout()){
  //   ROS_ERROR("[traj_server] Lost heartbeat from the planner, is he dead?");
  //   publish_cmd(last_mission_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_mission_yaw_, 0);
  //   return;
  // }

  ros::Time time_now = ros::Time::now();
  // Time elapsed since start of trajectory
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  // IF time elapsed is below duration of trajectory, then continue to send command
  if (t_cur >= 0.0 && t_cur < traj_duration_)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());

    // TODO: should we use time_forward_ here?
    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_->getPos(tf);

    time_last = time_now;
    last_mission_yaw_ = yaw_yawdot.first;
    last_mission_pos_ = pos;

    // publish PVA commands
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
  // IF time elapsed is longer then duration of trajectory, then nothing is done
  else if (t_cur >= traj_duration_) // Finished trajectory
  {
    // ROS_WARN_NAMED(node_name_, "t_cur exceeds traj_duration! No commands to send");
    endMission();
  }
  else {
    ROS_WARN_NAMED(node_name_, "Invalid time!");
  }
  
}

void TrajServer::startMission(){
  if (getServerState() == ServerState::MISSION){
    mission_completed_ = false;
  }
}

void TrajServer::endMission()
{
  mission_completed_ = true;
}

/* Conditional checking methods */

bool TrajServer::isLanded()
{
  // Check that difference between desired landing height and current UAV position
  // is within tolerance 
  return abs(uav_pose_.pose.position.z - landed_height_) < take_off_landing_tol_;
}

bool TrajServer::isTakenOff()
{
  // Check that difference between desired landing height and current UAV position
  // is within tolerance 
  return abs(uav_pose_.pose.position.z - takeoff_height_) < take_off_landing_tol_;
}

bool TrajServer::isMissionComplete()
{
  return mission_completed_;
}

bool TrajServer::isPlannerHeartbeatTimeout(){
  return (ros::Time::now() - heartbeat_time_).toSec() > planner_heartbeat_timeout_;
}

/* Helper methods */

bool TrajServer::toggle_offboard_mode(bool toggle)
  {
    bool arm_val = false;
    std::string set_mode_val = "AUTO.LOITER"; 
    if (toggle){
      arm_val = true;
      set_mode_val = "OFFBOARD"; 
    }

    auto conditions_fulfilled = [&] () {
      return (toggle ? is_uav_ready() : is_uav_idle());
    };

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = arm_val;

    mavros_msgs::SetMode set_mode_srv;
    set_mode_srv.request.custom_mode = set_mode_val;

    // Make sure takeoff is not immediately sent, 
    // this will help to stream the correct data to the program first.
    // Will give a 1sec buffer
    // ros::Duration(1.0).sleep();

    ros::Rate rate(pub_cmd_freq_);

    // send a few setpoints before starting
    for (int i = 0; ros::ok() && i < 10; i++)
    {
      execTakeOff();
      ros::spinOnce();
      rate.sleep();
    }
    ros::Time last_request_t = ros::Time::now();

    while (!conditions_fulfilled()){

      bool request_timeout = ((ros::Time::now() - last_request_t) > ros::Duration(2.0));

      if (uav_current_state_.mode != set_mode_val && request_timeout)
      {
        if (set_mode_client.call(set_mode_srv))
        {
          if (set_mode_srv.response.mode_sent){
            ROS_INFO_NAMED(node_name_, "Setting %s mode successful", set_mode_val.c_str());
          }
          else {
            ROS_INFO_NAMED(node_name_, "Setting %s mode failed", set_mode_val.c_str());
          }
        }
        else {
          ROS_INFO_NAMED(node_name_, "Service call to PX4 set_mode_client failed");
        }

        last_request_t = ros::Time::now();
      }
      else if (uav_current_state_.armed != arm_val && request_timeout) 
      {
        if (arming_client.call(arm_cmd)){
          if (arm_cmd.response.success){
            ROS_INFO_NAMED(node_name_, "Setting arm to %d successful", arm_val);
          }
          else {
            ROS_INFO_NAMED(node_name_, "Setting arm to %d failed", arm_val);
          }
        }
        else {
          ROS_INFO_NAMED(node_name_, "Service call to PX4 arming_client failed");
        }

        last_request_t = ros::Time::now();
      }
      ros::spinOnce();
      rate.sleep();        
    }

    return true;
  }

std::pair<double, double> TrajServer::calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_mission_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_mission_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_mission_yaw_dot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_mission_yaw_dot_ + dt * YDDM;
    d_yaw_max = last_mission_yaw_dot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_mission_yaw_dot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_mission_yaw_dot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_mission_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_mission_yaw_ = yaw_yawdot.first;
  last_mission_yaw_dot_ = yaw_yawdot.second;

  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

void TrajServer::publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double yaw, double yaw_rate, uint16_t type_mask)
{
  mavros_msgs::PositionTarget pos_cmd;

  pos_cmd.header.stamp = ros::Time::now();
  pos_cmd.header.frame_id = "world";
  pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  pos_cmd.type_mask = type_mask;
  // pos_cmd.type_mask = 3072; // Ignore Yaw
  // pos_cmd.type_mask = 2048; // ignore yaw_rate
  // pos_cmd.type_mask = 2496; // Ignore Acceleration
  // pos_cmd.type_mask = 3520; // Ignore Acceleration and Yaw
  // pos_cmd.type_mask = 2552; // Ignore Acceleration, Velocity, 
  // pos_cmd.type_mask = 3576; // Ignore Acceleration, Velocity and Yaw

  pos_cmd.position.x = p(0);
  pos_cmd.position.y = p(1);
  pos_cmd.position.z = p(2);
  pos_cmd.velocity.x = v(0);
  pos_cmd.velocity.y = v(1);
  pos_cmd.velocity.z = v(2);
  pos_cmd.acceleration_or_force.x = a(0);
  pos_cmd.acceleration_or_force.y = a(1);
  pos_cmd.acceleration_or_force.z = a(2);
  pos_cmd.yaw = yaw;
  pos_cmd.yaw_rate = yaw_rate;
  pos_cmd_raw_pub_.publish(pos_cmd);

  last_mission_pos_ = p;
  last_mission_yaw_ = yaw;
}

void TrajServer::setServerState(ServerState new_state)
{
  ROS_INFO_NAMED(node_name_, "Transitioning server state: %s -> %s", 
    StateToString(getServerState()).c_str(), StateToString(new_state).c_str());

  server_state_ = new_state;
}

ServerState TrajServer::getServerState()
{
  return server_state_;
}

void TrajServer::setServerEvent(ServerEvent event)
{
  ROS_INFO_NAMED(node_name_, "Set server event: %s", EventToString(event).c_str());

  server_event_ = event;
}

ServerEvent TrajServer::getServerEvent()
{
  // ROS_INFO_NAMED(node_name_, "Retrieved server event: %s", EventToString(server_event_).c_str());
  ServerEvent event = server_event_;
  server_event_ = ServerEvent::EMPTY_E;

  return event;
}
