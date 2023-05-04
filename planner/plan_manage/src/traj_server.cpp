#include <plan_manage/traj_server.h>

using namespace Eigen;

void TrajServer::init(ros::NodeHandle& nh)
{
  ROS_INFO_NAMED(node_name_, "Initializing");

  /* Params*/
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/pub_cmd_freq", pub_cmd_freq_, 25.0);
  nh.param("traj_server/state_machine_tick_freq", sm_tick_freq_, 50.0);
  nh.param("traj_server/take_off_height", takeoff_height_, 1.0);

  /* Subscribers */
  poly_traj_sub_ = nh.subscribe("planning/trajectory", 10, &TrajServer::polyTrajCallback, this);
  heartbeat_sub_ = nh.subscribe("heartbeat", 10, &TrajServer::heartbeatCallback, this);
  uav_state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &TrajServer::UAVStateCb, this);

  pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, &TrajServer::UAVPoseCB, this);

  set_server_state_sub_ = nh.subscribe<std_msgs::Int8>("/traj_server_event", 10, &TrajServer::serverEventCb, this);

  /* Publishers */
  pos_cmd_raw_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 50);
  pos_cmd_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 50);

  /* Service clients */
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  /* Timer callbacks */
  exec_traj_timer_ = nh.createTimer(ros::Duration(1/pub_cmd_freq_), &TrajServer::execTrajTimerCb, this);
  tick_sm_timer_ = nh.createTimer(ros::Duration(1/sm_tick_freq_), &TrajServer::tickServerStateTimerCb, this);
}

/* ROS Callbacks */

void TrajServer::heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void TrajServer::polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  ROS_INFO_NAMED(node_name_, "Trajectory received!");

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
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
}

void TrajServer::UAVStateCb(const mavros_msgs::State::ConstPtr &msg)
{
  ROS_INFO_THROTTLE_NAMED(5.0, node_name_, "State: Mode[%s], Connected[%d], Armed[%d]", msg->mode.c_str(), msg->connected, msg->armed);
  uav_current_state_ = *msg;
}

void TrajServer::UAVPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  uav_pose_ = *msg;

}

void TrajServer::serverEventCb(const std_msgs::Int8::ConstPtr & msg)
{
  if (msg->data < 0 || msg->data > ServerEvent::READY_FOR_MISSION){
    ROS_ERROR_NAMED(node_name_, "Invalid server event, ignoring...");
  } 
  setEvent(ServerEvent(msg->data));
}

void TrajServer::execTrajTimerCb(const ros::TimerEvent &e)
{
  switch (getServerState()){
    case ServerState::INIT:
      break;
    
    case ServerState::IDLE:
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
      execMissionTraj();
      break;
  }

}

void TrajServer::tickServerStateTimerCb(const ros::TimerEvent &e)
{
  switch (server_state_)
  {
    case ServerState::INIT:
      ROS_INFO_NAMED(node_name_, "[INIT] Initializing Server");

      ros::Rate rate(1/pub_cmd_freq_);
      // Wait for FCU Connection
      while (ros::ok() && !uav_current_state_.connected)
      { 
        ROS_WARN_THROTTLE_NAMED(1, node_name_, "Waiting for connection to FCU...");
        ros::spinOnce();
        rate.sleep();
      }

      setServerState(ServerState::IDLE);
      break;
    
    case ServerState::IDLE:
      ROS_INFO_THROTTLE_NAMED(1.0, node_name_, "[IDLE] Ready to take off");
      switch (getEvent())
      {
        case TAKEOFF:
          ROS_INFO_NAMED(node_name_, "Attempting takeoff");
          
          break;
        case LAND:
          ROS_WARN_NAMED(node_name_, "Agent has not taken off, unable to LAND");
          break;
        case READY_FOR_MISSION:
          ROS_WARN_NAMED(node_name_, "Please TAKEOFF first before setting MISSION mode");
          break;
        case EMPTY:
          // Default case if no event sent
          break;
      }
      break;
    
    case ServerState::TAKEOFF:
      ROS_INFO_NAMED(node_name_, "[TAKEOFF] Taking off");
      if (!is_uav_ready()){
        enable_offboard_mode();
      }
      // TODO: Use a global timer to determine if takeoff been completed

      // TODO: Send take off command

      // TODO: Handle events

      ROS_INFO_NAMED(node_name_, "[TAKEOFF] Take off complete");
      setServerState(ServerState::HOVER);
      break;
    
    case ServerState::LAND:
      ROS_INFO_NAMED(node_name_, "[LAND] Landed");

      // TODO: Land command

      setServerState(ServerState::IDLE);
      break;
    
    case ServerState::HOVER:
      ROS_INFO_NAMED(node_name_, "[HOVER] Ready to receive trajectories");

      // TODO: Keep hovering

      // TODO: Handle events

      break;
    
    case ServerState::MISSION:
      ROS_INFO_NAMED(node_name_, "[MISSION] Executing trajectory");

      // TODO: Handle events

      break;
  }
}


void TrajServer::execLand()
{

}

void TrajServer::execTakeOff()
{ 
  mavros_msgs::PositionTarget pos_cmd;

  pos_cmd.header.stamp = ros::Time::now();
  pos_cmd.header.frame_id = "world";
  pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  pos_cmd.type_mask = 3576; // Ignore Velocity, Acceleration and Yaw
  // pos_cmd.type_mask = 2552; // Ignore Velocity, Acceleration
  // pos_cmd.type_mask = 2496; // Ignore Acceleration
  // pos_cmd.type_mask = 3520; // Ignore Acceleration and Yaw
  // pos_cmd.type_mask = 3072; // Ignore Yaw
  // pos_cmd.type_mask = 2048; // ignore v,a, yaw and yaw_rate

  pos_cmd.position = uav_pose_.pose.position;
  pos_cmd.position.z = takeoff_height_;

  pos_cmd_raw_pub_.publish(pos_cmd);
}

void TrajServer::execHover()
{

}

void TrajServer::execMissionTraj()
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_){
    // ROS_WARN_THROTTLE_NAMED(1, node_name_, "No trajectory received");
    return;
  }

  ros::Time time_now = ros::Time::now();

  // If heartbeat timeout, send stop command
  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is he dead?");

    receive_traj_ = false;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
  }

  // Time elapsed since start of trajectory
  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  // IF time elapsed is below duration of trajectory, then continue to send command
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    ROS_INFO_NAMED(node_name_, "Sending traj");

    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());

    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_->getPos(tf);

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    // publish PVA commands
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
  // IF time elapsed is longer then duration of trajectory, then nothing is done
  else if (t_cur >= traj_duration_) // Finished trajectory
  {
    receive_traj_ = false;
    // ROS_WARN_NAMED(node_name_, "t_cur exceeds traj_duration! No commands to send");
  }
  else {
    ROS_WARN_NAMED(node_name_, "Invalid time!");
  }
  
}

/* Helper methods */

void TrajServer::setServerState(ServerState new_state)
{
  ROS_INFO_NAMED(node_name_, "Transitioning server state: %s -> %s", 
    StateToString(server_state_), StateToString(new_state));

  server_state_ = new_state;
}

ServerState TrajServer::getServerState()
{
  return server_state_;
}

void TrajServer::setEvent(ServerEvent event)
{
  ROS_INFO_NAMED(node_name_, "Set server event: %s", EventToString(event));

  server_event_ = event;
}

ServerEvent TrajServer::getEvent()
{
  ROS_INFO_NAMED(node_name_, "Retrieved server event: %s", EventToString(server_event_));
  ServerEvent event = server_event_;
  server_event_ = ServerEvent::EMPTY;

  return event;
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
                        : last_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
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
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

void TrajServer::publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double yaw, double yaw_rate)
{
  mavros_msgs::PositionTarget pos_cmd;

  pos_cmd.header.stamp = ros::Time::now();
  pos_cmd.header.frame_id = "world";
  pos_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  // pos_cmd.type_mask = 2048; // use p,v,a and ignore yaw_rate

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

  last_pos_ = p;
}
