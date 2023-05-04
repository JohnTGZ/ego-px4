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

using std::vector;

namespace ego_planner
{

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

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code

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
    bool flag_realworld_experiment_;
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
    // int formation_num_;
    Eigen::Vector3d formation_pos_;

    /* ROS utils */
    ros::NodeHandle node_;
    // Timer to execute FSM callback
    ros::Timer exec_timer_;
    ros::Timer safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
    ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_;

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
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    // void planGlobalTrajbyGivenWps();
    void readGivenWpsAndPlan();
    void planNextWaypoint(const Eigen::Vector3d next_wp, const Eigen::Vector3d previous_wp);

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

    /* ground height measurement */
    bool measureGroundHeight(double &height);

    // Count number of FSM exec iterations
    int fsm_itr_num{0};

    std::pair<bool,bool> isGoalReachedAndReplanNeeded();
  };

} // namespace ego_planner

#endif