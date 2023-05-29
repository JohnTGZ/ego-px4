
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    have_trigger_ = true;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* ROS Params*/

    // This applies an offset to all trajectories received and sent so that they are relative to the world frame
    nh.param("fsm/frame_offset_x", uav_origin_to_world_tf_.position.x, 0.0);
    nh.param("fsm/frame_offset_y", uav_origin_to_world_tf_.position.y, 0.0);
    nh.param("fsm/frame_offset_z", uav_origin_to_world_tf_.position.z, 0.0);

    // Reverse signs so that the transformation is from UAV origin frame to world frame
    world_to_uav_origin_tf_.position.x = -uav_origin_to_world_tf_.position.x;
    world_to_uav_origin_tf_.position.y = -uav_origin_to_world_tf_.position.y;
    world_to_uav_origin_tf_.position.z = -uav_origin_to_world_tf_.position.z;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);

    int formation_num = -1;
    nh.param("formation/num", formation_num, -1);
    if (formation_num < planner_manager_->pp_.drone_id + 1)
    {
      logError("formation_num is smaller than the drone number, illegal!");
      return;
    }
    std::vector<double> pos;
    nh.getParam("formation/drone" + to_string(planner_manager_->pp_.drone_id), pos);
    formation_pos_ << pos[0], pos[1], pos[2];
    nh.getParam("formation/start", pos);
    formation_start_ << pos[0], pos[1], pos[2];

    /* Timer callbacks */
    tick_state_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::tickStateTimerCB, this);
    exec_state_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::execStateTimerCB, this);

    /* Subscribers */
    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    // Use MINCO trajectory to minimize the message size in wireless communication
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

    /* Publishers */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);


    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      // Subscribe to waypoints 
      waypoints_sub_ = nh.subscribe("/waypoints", 1, &EGOReplanFSM::waypointsCB, this);
    }
    else{
      logError(string_format("Wrong target_type_ value! target_type_=%i", target_type_));
    }

  }

  /**
   * Timer Callbacks
  */

  void EGOReplanFSM::tickStateTimerCB(const ros::TimerEvent &e)
  {
    // logInfoThrottled(string_format("Current State: [%s]", StateToString(getServerState()).c_str()), 1.0);
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    if (fsm_num++ == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (getServerState())
    {
      case INIT:
      {
        if (have_odom_){
          setServerState(WAIT_TARGET);
        }
        break;
      }

      case WAIT_TARGET: // Wait for target or trigger
      {
        if (have_target_ && have_trigger_) {
          setServerState(SEQUENTIAL_START);
        }
        break;
      }

      case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
      {
        switch (getServerEvent())
        {
          case WAIT_TARGET_E:
            break;
          case GEN_NEW_TRAJ_E:
            break;
          case REPLAN_TRAJ_E:
            break;
          case EXEC_TRAJ_E:
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          case SEQUENTIAL_START_E:
            break;
          case EMPTY_E:
            // Default case if no event sent
            break;
        }

        break;
      }

      case GEN_NEW_TRAJ:
      {
        switch (getServerEvent())
        {
          case WAIT_TARGET_E:
            break;
          case GEN_NEW_TRAJ_E:
            break;
          case REPLAN_TRAJ_E:
            break;
          case EXEC_TRAJ_E:
            setServerState(ServerState::EXEC_TRAJ);
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          case SEQUENTIAL_START_E:
            break;
          case EMPTY_E:
            // Default case if no event sent
            break;
        }

        break;
      }

      case REPLAN_TRAJ:
      {

        switch (getServerEvent())
        {
          case WAIT_TARGET_E:
            break;
          case GEN_NEW_TRAJ_E:
            break;
          case REPLAN_TRAJ_E:
            break;
          case EXEC_TRAJ_E:
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          case SEQUENTIAL_START_E:
            break;
          case EMPTY_E:
            // Default case if no event sent
            break;
        }

        break;
      }

      case EXEC_TRAJ:
      {
        switch (getServerEvent())
        {
          case WAIT_TARGET_E:
            setServerState(ServerState::WAIT_TARGET);
            break;
          case GEN_NEW_TRAJ_E:
            break;
          case REPLAN_TRAJ_E:
            setServerState(ServerState::REPLAN_TRAJ);
            break;
          case EXEC_TRAJ_E:
            break;
          case EMERGENCY_STOP_E:
            logFatal("EMERGENCY STOP ACTIVATED!");
            setServerState(ServerState::EMERGENCY_STOP);
            break;
          case SEQUENTIAL_START_E:
            break;
          case EMPTY_E:
            // Default case if no event sent
            break;
        }

        break;
      }

      case EMERGENCY_STOP:
      {
        if (flag_escape_emergency_) // Avoiding repeated calls to callEmergencyStop
        {
          callEmergencyStop(odom_pos_);
        }
        else
        {
          if (enable_fail_safe_ && odom_vel_.norm() < 0.1){
            setServerState(GEN_NEW_TRAJ);
          }
        }

        flag_escape_emergency_ = false;
        break;
      }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  void EGOReplanFSM::execStateTimerCB(const ros::TimerEvent &e){
    switch (getServerState())
    {
      case INIT:
      {
        break;
      }

      case WAIT_TARGET: 
      {
        break;
      }

      case SEQUENTIAL_START: 
      {
        // If first drone or it has received the trajectory of the previous agent.
        if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
        {
          if (planFromGlobalTraj(10))
          {
            setServerState(EXEC_TRAJ);
          }
          else
          {
            logError("Failed to generate the first global trajectory!!!");
            setServerState(SEQUENTIAL_START); // "setServerState" must be called each time planned
          }
        }

        break;
      }

      case GEN_NEW_TRAJ:
      {
        if (planFromGlobalTraj(10)) // Try planning 10 times
        {
          setServerState(EXEC_TRAJ); 
          flag_escape_emergency_ = true; // TODO Refactor
        }

        break;
      }

      case REPLAN_TRAJ:
      {
        if (planFromLocalTraj(1))
        {
          setServerState(EXEC_TRAJ);
        }

        break;
      }

      case EXEC_TRAJ:
      {
        ServerEvent triggered_event = safetyChecks();
        setServerEvent(triggered_event);

        std::pair<bool,bool> GoalReachedAndReplanNeededCheck = isGoalReachedAndReplanNeeded();

        if (GoalReachedAndReplanNeededCheck.first) {
          // The navigation task completed 
          setServerEvent(WAIT_TARGET_E);
        }
        else if (GoalReachedAndReplanNeededCheck.second) {
          // Replanning trajectory needed 
          setServerEvent(REPLAN_TRAJ_E);
        }

        break;
      }

      case EMERGENCY_STOP:
      {
        break;
      }
    }
  }

  /**
   * Subscriber Callbacks
  */
  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    logError("Received a mandatory stop command!");
    setServerState(EMERGENCY_STOP);
    setServerEvent(EMERGENCY_STOP_E);
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x ;
    odom_pos_(1) = msg->pose.pose.position.y ;
    odom_pos_(2) = msg->pose.pose.position.z ;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    logInfo("Execution of goals triggered!");
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    traj_utils::MINCOTraj minco_traj = *msg;

    const size_t recv_id = (size_t)minco_traj.drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id){ // Receiving the same plan produced by this very drone
      return;
    }

    if (minco_traj.drone_id < 0)
    {
      logError("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (minco_traj.order != 5)
    {
      logError("Only support trajectory order equals 5 now!");
      return;
    }
    if (minco_traj.duration.size() != (minco_traj.inner_x.size() + 1))
    {
      logError("Wrong trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        minco_traj.start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      logWarn(string_format("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id));
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - minco_traj.start_time).toSec()) > 0.25)
    {

      if (abs((t_now - minco_traj.start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        logWarn(string_format("Time stamp diff: Local - Remote Agent %d = %fs",
                 minco_traj.drone_id, (t_now - minco_traj.start_time).toSec()));
      }
      else
      {
        logError(string_format("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  minco_traj.drone_id, (t_now - minco_traj.start_time).toSec()));
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    transformMINCOTrajectoryToUAVOrigin(minco_traj);

    /* Store data */
    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = minco_traj.traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = minco_traj.start_time.toSec();

    int piece_nums = minco_traj.duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;

    // Position here is transformed to UAV frame 
    headState <<  minco_traj.start_p[0], 
                    minco_traj.start_v[0], minco_traj.start_a[0],
                  minco_traj.start_p[1], 
                    minco_traj.start_v[1], minco_traj.start_a[1],
                  minco_traj.start_p[2], 
                    minco_traj.start_v[2], minco_traj.start_a[2];
    tailState <<  minco_traj.end_p[0], 
                    minco_traj.end_v[0], minco_traj.end_a[0],
                  minco_traj.end_p[1], 
                    minco_traj.end_v[1], minco_traj.end_a[1],
                  minco_traj.end_p[2], 
                    minco_traj.end_v[2], minco_traj.end_a[2];

    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);

    for (int i = 0; i < piece_nums - 1; i++){
      innerPts.col(i) <<  minco_traj.inner_x[i], 
                          minco_traj.inner_y[i], 
                          minco_traj.inner_z[i];
    }
    for (int i = 0; i < piece_nums; i++){
      durations(i) = minco_traj.duration[i];
    }

    // Optimize for minimum jerk
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    poly_traj::Trajectory trajectory = MJO.getTraj();
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    /* Check Collision */
    if (planner_manager_->checkCollision(recv_id))
    {
      // TODO: What state is it expected to be in? EXEC_TRAJ?
      setServerEvent(REPLAN_TRAJ_E);
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      // If the number of trajectories exceed or are same as current drone id
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        // For each drone id
        for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
        {
          // If the drone_id of the i-th swarm trajectory is not the same as the i-th value, then break out
          if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
          {
            break;
          }
          have_recv_pre_agent_ = true;
        }
      }
    }
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    // TODO: Refactor receiving of wayopints into a standardized function
    
    Eigen::Vector3d next_wp(
      msg->pose.position.x + world_to_uav_origin_tf_.position.x, 
      msg->pose.position.y + world_to_uav_origin_tf_.position.y, 
      1.0 + world_to_uav_origin_tf_.position.z);
    
    // Add latest waypoint to waypoint list
    wps_.push_back(next_wp);
    // Set waypoint id as the latest one
    wpt_id_ = wps_.size() - 1;
    // If there are at least 2 waypoints,
    if (wpt_id_ >= 1)
    {
      // Set the formation_start to be second last waypoint 
      formation_start_ = wps_[wpt_id_ - 1];
    }

    // Plan from formation_start_ to final waypoint
    planNextWaypoint(wps_[wpt_id_], formation_start_);
  }

  void EGOReplanFSM::waypointsCB(const trajectory_server_msgs::WaypointsPtr &msg)
  {
    if (!have_odom_)
    {
      logError("No odom received, rejecting waypoints!");
      return;
    }
    if (msg->waypoints.poses.size() <= 0)
    {
      logError("Received empty waypoints");
      return;
    }
    if (msg->waypoints.header.frame_id != "world")
    {
      logError("Only waypoint goals in 'world' frame are accepted, ignoring waypoints.");
      return;
    }

    waypoint_num_ = msg->waypoints.poses.size();

    wps_.clear();
    // Transform received waypoints from world to UAV origin frame
    for (auto pose : msg->waypoints.poses) {
      wps_.push_back(Eigen::Vector3d{
        pose.position.x + world_to_uav_origin_tf_.position.x,
        pose.position.y + world_to_uav_origin_tf_.position.y,
        pose.position.z + world_to_uav_origin_tf_.position.z
      });
    }

    for (size_t i = 0; i < wps_.size(); i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wpt_id_ = 0;
    // Plan from formation_start_ to first waypoint
    planNextWaypoint(wps_[wpt_id_], formation_start_);
  }

  /**
   * Planning Methods
  */
  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    // If this is the first time planning has been called, then initialize a random polynomial
    bool flag_random_poly_init = (timesOfConsecutiveStateCalls().first == 1);

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    // Try replanning (2 + 'trial_times') number of times.
    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // Get local target position and velocity 
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    // replan from 'formation_start_' to 'wps_[wpt_id_]'
    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, 
        start_acc_, local_target_pt_, 
        local_target_vel_, formation_start_, 
        wps_[wpt_id_], (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    if (plan_success)
    {
      // Get data from local trajectory and store in PolyTraj and MINCOTraj 
      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;

      polyTraj2ROSMsg(poly_msg, MINCO_msg);

      poly_traj_pub_.publish(poly_msg); // Publish to corresponding drone for execution
      broadcast_ploytraj_pub_.publish(MINCO_msg); // Broadcast to all other drones for replanning to optimize in avoiding swarm collision
    }

    return plan_success;
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp, const Eigen::Vector3d previous_wp)
  {

    Eigen::Vector3d dir = (next_wp - previous_wp).normalized();
    // Offset end_pt_ by the formation position
    end_pt_ = next_wp + Eigen::Vector3d(dir(0) * formation_pos_(0) - dir(1) * formation_pos_(1),
                                        dir(1) * formation_pos_(0) + dir(0) * formation_pos_(1),
                                        formation_pos_(2));

    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      // gloabl_traj is only used for visualization
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;
      // have_trigger_ = true;

      /*** FSM ***/

      if (getServerState() != WAIT_TARGET)
      {
        // If already executing a trajectory then wait for it to finish
        while (getServerState() != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        // If not in WAIT_TARGET state, Go to REPLAN_TRAJ
        setServerState(REPLAN_TRAJ);
        setServerEvent(REPLAN_TRAJ_E);
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      logError("Unable to generate global trajectory! Undefined actions!");
    }
  }

  std::pair<bool,bool> EGOReplanFSM::isGoalReachedAndReplanNeeded(){
    // boolean values to denote current state of plan execution
    bool goal_reached{false}, replan_needed{false};

    /* determine if need to replan */
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    t_cur = min(info->duration, t_cur);

    // TODO: use actual position 
    // Get position at current time
    Eigen::Vector3d pos = info->traj.getPos(t_cur);

    // Local target point and final goal is close enough
    bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);

    //TODO remove target_type_ == TARGET_TYPE::PRESET_TARGET condition 
    // so that there is no difference between manually provided goals and waypoints

    // SAME STATE: Plan next waypoint if:
    // 1. current waypoint id is not the last waypoint
    // 2. distance between current pos and end point is below no_replan_thresh_ 
    if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
        (wpt_id_ < waypoint_num_ - 1) &&
        (end_pt_ - pos).norm() < no_replan_thresh_)
    {
      formation_start_ = wps_[wpt_id_];
      wpt_id_++;
      planNextWaypoint(wps_[wpt_id_], formation_start_);
    }
    // GOAL REACHED: local target close to the global target
    else if ((t_cur > info->duration - 1e-2) && touch_the_goal) 
    {
      // Restart the target and trigger
      have_target_ = false;

      // TODO: Disabled trigger here
      // have_trigger_ = false;

      if (target_type_ == TARGET_TYPE::PRESET_TARGET)
      {
        // Plan from start of formation to first waypoint
        formation_start_ = wps_[wpt_id_];
        wpt_id_ = 0;
        planNextWaypoint(wps_[wpt_id_], formation_start_);
        // have_trigger_ = false; // must have trigger
      }

      /* The navigation task completed */
      goal_reached = true;
      replan_needed = false;
    }
    // GOAL REACHED if:
    // 1. distance between current pos and end point is below no_replan_thresh_ 
    // 2. Goal is inside obstacle 
    else if ((end_pt_ - pos).norm() < no_replan_thresh_ &&
              planner_manager_->grid_map_->getInflateOccupancy(end_pt_))
    {
      have_target_ = false;
      have_trigger_ = false;
      logError("The goal is in obstacles, performing an emergency stop.");
      callEmergencyStop(odom_pos_);

      /* The navigation task completed */
      goal_reached = true;
      replan_needed = false;
    }
    // REPLAN: if current time elapsed exceeds replan time threshold
    else if (t_cur > replan_thresh_ ||
            (!touch_the_goal && planner_manager_->traj_.local_traj.pts_chk.back().back().first - t_cur < emergency_time_))
    {
      // Replan trajectory needed
      goal_reached = false;
      replan_needed = true;
    }

    return std::make_pair(goal_reached, replan_needed);
  }

  EGOReplanFSM::ServerEvent EGOReplanFSM::safetyChecks()
  {
    /* Perform safety checks
      1. Check ground height
      2. Check for loss of sensor data
      3. Check for sufficient clearance between current agent's trajectories and 
      that of others' 
    */

    // 1. check ground height 
    // TODO: Implement ground height checking
    double height;
    measureGroundHeight(height);

    // Get local trajectory
    LocalTrajData *info = &planner_manager_->traj_.local_traj;

    // Return if no local trajectory yet
    if (info->traj_id <= 0){
      return EMPTY_E;
    }
    auto map = planner_manager_->grid_map_;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk; // Points to use for checking

    // 2. Check for loss of sensor data
    if (map->getOdomDepthTimeout())
    {
      logError("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      setServerState(EMERGENCY_STOP);
      return EMERGENCY_STOP_E;
    }

    // 3. Check trajectory clearance
    const double CLEARANCE = 0.8 * planner_manager_->getSwarmClearance();
    auto id_ratio = info->traj.locatePieceIdxWithRatio(t_cur);

    // cout << "t_cur=" << t_cur << " info->duration=" << info->duration << endl;

    // i_start is piece index
    size_t i_start = floor((id_ratio.first + id_ratio.second) * planner_manager_->getCpsNumPrePiece());

    if (i_start >= pts_chk.size())
    {
      logError("i_start >= pts_chk.size()");
      return EMPTY_E;
    }

    size_t j_start = 0;
    // cout << "i_start=" << i_start << " pts_chk.size()=" << pts_chk.size() << " pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
    for (; i_start < pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
    
    find_ij_start:;

      // Is local target near the goal?
      const bool target_near_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);
      size_t i_end = target_near_goal ? pts_chk.size() : pts_chk.size() * 3 / 4;
      for (size_t i = i_start; i < i_end; ++i)
      {
        for (size_t j = j_start; j < pts_chk[i].size(); ++j)
        {

          double t = pts_chk[i][j].first; // time
          Eigen::Vector3d pos = pts_chk[i][j].second; //position
          bool occ = false; // Indicates if occupancy grid is occupied
          occ |= map->getInflateOccupancy(pos);

          // Iterate through trajectories of other agents
          for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
          {
            // Skip own trajectory or if drone ID of trajectory does not match desired ID
            if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
                (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
            {
              continue;
            }

            // Calculate time for other drone
            double t_X = t - (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
            if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration) // If valid time
            {
              Eigen::Vector3d agent_predicted_pos = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
              double dist = (pos - agent_predicted_pos).norm();

              if (dist < CLEARANCE)
              {
                logWarn(string_format("Clearance between drone %d and drone %d is %f, too close!",
                        planner_manager_->pp_.drone_id, (int)id, dist));
                occ = true;
                break;
              }
            }
          }

          if (occ)
          {
            logInfo(string_format("Replanning to avoid collision."));
            if (planFromLocalTraj()) 
            {
              logInfo(string_format("Replanning successful to avoid collision. %f", t / info->duration));
              setServerState(EXEC_TRAJ);
              return EXEC_TRAJ_E;
            }
            else
            {
              logError(string_format("Replan failed upon detecting potential collision"));
              if (t - t_cur < emergency_time_) 
              {
                logWarn(string_format("Emergency stop! time=%f", t - t_cur));
                setServerState(EMERGENCY_STOP);
                return EMERGENCY_STOP_E;
              }
              else
              {
                logWarn("Current trajectory in collision, replanning.");
                setServerState(REPLAN_TRAJ);
                return REPLAN_TRAJ_E;
              }
            }
            break;
          }
        }
        j_start = 0;
      }
  }

  /**
   * Helper Methods
  */
 
  // display the FSM state along with other indicators (have_odom, have_target, have_trigger etc.)
  void EGOReplanFSM::printFSMExecState()
  {
    std::string msg{""};
    msg += string_format("[FSM]: state: %s", StateToString(getServerState()).c_str());

    if (!have_odom_)
    {
      msg += ", waiting for odom";
    }
    if (!have_target_)
    {
      msg += ", waiting for target";
    }
    if (!have_trigger_)
    {
      msg += ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      msg += ", haven't receive traj from previous drone";
    }

    logInfo(msg);
  }

  std::pair<int, EGOReplanFSM::ServerState> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, ServerState>(continously_called_times_, getServerState());
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {
    // Get local trajectory
    auto data = &planner_manager_->traj_.local_traj;

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);

    // For each point
    for (int i = 0; i < piece_num; ++i)
    {
      // Assign timestamp
      poly_msg.duration[i] = durs(i);

      // Assign coefficient matrix values
      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);

    Eigen::Vector3d vec; // Vector representing x,y,z values or their derivatives
    // Start Position
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    // Start Velocity
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    // Start Acceleration
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    // End position
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    // End velocity
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    // End Acceleration
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);

    // Assign inner points
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++){
      MINCO_msg.duration[i] = durs[i];
    }
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;

    polyTraj2ROSMsg(poly_msg, MINCO_msg);

    transformMINCOTrajectoryToWorld(MINCO_msg);

    poly_traj_pub_.publish(poly_msg); // Publish to corresponding drone for execution
    broadcast_ploytraj_pub_.publish(MINCO_msg); // Broadcast to all other drones for replanning to optimize in avoiding swarm collision

    return true;
  }

} // namespace ego_planner
