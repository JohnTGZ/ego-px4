//
// Copyright (c) 2016-2020 Kris Jusiak (kris at jusiak dot net)
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
#include "sml.hpp"
#include <cassert>
#include <iostream>
// #include <chrono>
// #include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace sml = boost::sml;

namespace {
// class Planner {
// using self = Planner;
// public:

//   // Events
//   struct GetPlan{};

//   // Guards

//   // States
//   struct IDLE{};
//   struct PLANNING{};


// };

// ----------------------------------------------------------------------------
// Event Declarations
//
struct FSMUpdate{
  bool have_odom{false};
  bool have_target{false};
  bool have_trigger{false};
  bool flag_escape_emergency{false};
  bool plan_success{false};

  bool have_recv_pre_agent{false}; // All drones have assigned trajectories
};
struct TriggerExec{
};

auto FSMUpdateEvent = sml::event<FSMUpdate>;
auto TriggerExecEvent = sml::event<TriggerExec>;

// ----------------------------------------------------------------------------
// States
// 
auto INIT = sml::state<class INIT>;
auto WAIT_TARGET = sml::state<class WAIT_TARGET>;
auto SEQUENTIAL_START = sml::state<class SEQUENTIAL_START>;
auto EXEC_TRAJ = sml::state<class EXEC_TRAJ>;

class EgoFSM {
using self = EgoFSM;

public:

  EgoFSM() {
    ROS_INFO("Constructed EgoFSM");
  }

  // ----------------------------------------------------------------------------
  // Guards
  // 

  /**
   * Check if it is possible to transition to WAIT_TARGET state
  */
  bool wait_target_guard(const FSMUpdate& event) const noexcept { 
    if (event.have_odom){
      return true;
    }
    else {
      // goto force_return; // return;
      return false;
    }
  }

  /**
   * Check if it is possible to transition to SEQUENTIAL_START state
  */
  bool seq_start_guard(const FSMUpdate& event) { 
    std::cout << "seq_start_guard" << std::endl;

    return true;

    // if (event.have_target && event.have_trigger){
    //   return true;
    // }
    // else {
    //   return false;
    // }
  }

  /**
   * Check if it is possible to transition to EXEC_TRAJ state
  */
  bool exec_traj_guard() { 
    std::cout << "exec_traj_guard" << std::endl;
    if (fsm_update_.plan_success ){
      return true;
    }
    else {
      return false;
    }
  }

  auto operator()() const noexcept{
    using namespace sml;

    return make_transition_table(
      *INIT + FSMUpdateEvent [ &self::wait_target_guard ]  = WAIT_TARGET,

      WAIT_TARGET + sml::on_entry<_> / &self::enable_offboard_mode, 
      WAIT_TARGET + FSMUpdateEvent [ &self::seq_start_guard ] = SEQUENTIAL_START,

      // SEQUENTIAL_START + sml::on_entry<_> / &self::planFromGlobalTraj, 
      // SEQUENTIAL_START + FSMUpdateEvent / &self::planFromGlobalTraj,
      // SEQUENTIAL_START + FSMUpdateEvent / (&self::planFromGlobalTraj, [](auto, auto&sm){ sm.process_event(TriggerExec{});}),

      SEQUENTIAL_START + sml::on_entry<_> / &self::planFromGlobalTraj, 
      SEQUENTIAL_START + FSMUpdateEvent [ &self::exec_traj_guard ] / &self::planFromGlobalTraj = EXEC_TRAJ

      // SEQUENTIAL_START + FSMUpdateEvent / (&self::planFromGlobalTraj, process(TriggerExec{})),

      // SEQUENTIAL_START + TriggerExecEvent [ &self::exec_traj_guard ] = EXEC_TRAJ
    );
  }

  /* Planning Methods */
  void planFromGlobalTraj(const int trial_times) 
  {
    std::cout << "Planning Global Traj!" << std::endl;
    fsm_update_.plan_success = true;
  }
  
  /* Helper Methods */
  void enable_offboard_mode()
  {
    std::cout << "Enabling offboard mode!" << std::endl;
  }

  // To be updated by ROS Callbacks 
  FSMUpdate fsm_update_;

  // Parameters
  int drone_id{0};
};


class TaskMaster{
public:

  void init(ros::NodeHandle& nh){
    /* callback */
    odom_sub_ = nh.subscribe("odom_world", 1, &TaskMaster::odometryCb, this);

    /* timers */
    fsm_update_timer_ = nh.createTimer(ros::Duration(1.0), &TaskMaster::FSMUpdateTimerCb, this);
    ROS_INFO("Initialized EgoFSM");

    // Create state machine
    ego_fsm_.reset(new EgoFSM());

    // ego_fsm_->fsm_update_.have_target = false;
    // ego_fsm_->fsm_update_.have_trigger = false;
    // ego_fsm_->fsm_update_.have_odom = false;
    // ego_fsm_->fsm_update_.have_recv_pre_agent = false;
    // ego_fsm_->fsm_update_.flag_escape_emergency = true;

    sm_ptr_.reset(new sml::sm<EgoFSM>{*ego_fsm_});
  }

  /* ROS Callbacks */
  void odometryCb(const nav_msgs::OdometryConstPtr &msg) {
    ego_fsm_->fsm_update_.have_odom = true;
  }

  void FSMUpdateTimerCb(const ros::TimerEvent &e) {
    sm_ptr_->process_event(ego_fsm_->fsm_update_);
    print_current_state();
  }

  /* Helper Methods */

  // Print current state of FSM
  void print_current_state() {
    // TODO Perhaps use a std::map to solve this problem
    std::string current_state{""};

    if (sm_ptr_->is(INIT)){
      current_state = "INIT";
    }
    else if (sm_ptr_->is(WAIT_TARGET)){
      current_state = "WAIT_TARGET";
    }
    else if (sm_ptr_->is(SEQUENTIAL_START)) {
      current_state = "SEQUENTIAL_START";
    }
    else if (sm_ptr_->is(EXEC_TRAJ)) {
      current_state = "EXEC_TRAJ";
    }
    else {
      current_state = "UNDEFINED";
    }

    std::cout << "Current state: " << current_state << std::endl;
  }

  // Subscribers
  ros::Subscriber odom_sub_;

  // Timer to execute FSM Update callback
  ros::Timer fsm_update_timer_;

  // State machine class
  std::unique_ptr<EgoFSM> ego_fsm_;
  // State machine controller
  std::unique_ptr<sml::sm<EgoFSM>> sm_ptr_;
};

} // namespace

int main(int argc, char** argv) {

  using namespace sml;

  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  TaskMaster task_master;
  task_master.init(nh);

  ros::spin();

  return 0;

  // task_master.fsm_update_.have_odom = true;
  // sm.process_event(task_master.fsm_update_);
  // // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // assert(sm.is(state<EgoFSM::WAIT_TARGET>));

  // task_master.fsm_update_.have_target = true;
  // task_master.fsm_update_.have_trigger = true;
  // sm.process_event(task_master.fsm_update_);
  // assert(sm.is(sml::state<SEQUENTIAL_START>));
  
  // sm.process_event(task_master.fsm_update_);
  // assert(sm.is(sml::state<EXEC_TRAJ>));

  // sm.process_event(timeout{});
  // assert(sm.is(sml::X));  // released

  std::cout << "hello_world terminated!" << std::endl;

}


// int main() {
// }