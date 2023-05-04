#include "tinyfsm.hpp"
#include <iostream>
#include <cassert>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

class TaskMaster {
public:
  TaskMaster() {
    ROS_INFO("Constructed TaskMaster");
  }

  void init(ros::NodeHandle& nh){
    state_update_event_.have_target = false;
    state_update_event_.have_trigger = false;
    state_update_event_.have_odom = false;
    state_update_event_.have_recv_pre_agent = false;
    state_update_event_.flag_escape_emergency = true;

    /* callback */
    odom_sub_ = nh.subscribe("odom_world", 1, &TaskMaster::odometryCb, this);

    /* timers */
    fsm_update_timer_ = nh.createTimer(ros::Duration(1.0), &TaskMaster::FSMUpdateTimerCb, this);
    ROS_INFO("Initialized TaskMaster");

    // Start state machine
    TaskMaster::PlannerFSM::start();
  }

  // 1. Event Declarations
  struct StateUpdateEvent : tinyfsm::Event {
    bool have_target; // Target goal received
    bool have_trigger;
    bool have_odom;
    bool flag_escape_emergency;

    bool have_recv_pre_agent; // All drones have assigned trajectories
  };

  // 2. State Machine Base Class Declaration
  class PlannerFSM : public tinyfsm::Fsm<PlannerFSM>
  {
  public:
    virtual void react(StateUpdateEvent const &) {};
    // alternative: enforce handling of Toggle in all states (pure virtual)
    // virtual void react(StateUpdateEvent const &) = 0;

    virtual void entry(void) {};  /* entry actions in some states */
    void exit(void) {}; /* no exit actions*/

    int current_state = 0;
  };

  // 3. State Declarations
  class INIT : public PlannerFSM
  {
    void entry() override {
      std::cout << "Entered INIT" << std::endl;
    }

    void react(StateUpdateEvent const& event) override {
      if (!event.have_odom) {
        // goto force_return; // return;
      }
      else {
        transit<WAIT_TARGET>();
      }
    }

    const std::string state_name_{"Init"}; 
  };

  class WAIT_TARGET : public PlannerFSM
  {
    void entry() override {
      std::cout << "Entered WAIT_TARGET" << std::endl;
    }

    void react(StateUpdateEvent const& event) override {
      if (event.have_target && event.have_trigger){
        transit<SEQUENTIAL_START>();
      }
      else {
        // goto force_return; // return;
      }
    }

    const std::string state_name_{"Init"}; 
  };

  class SEQUENTIAL_START : public PlannerFSM
  {
    void entry() override {
      std::cout << "Entered SEQUENTIAL_START" << std::endl;
    }

    void react(StateUpdateEvent const& event) override {
      if (event.have_recv_pre_agent) {
        // if (planFromGlobalTraj(10)) {
        //   transit<EXEC_TRAJ>();
        // }
        // else {
        //   transit<SEQUENTIAL_START>();
        // }
      }
    }

    const std::string state_name_{"Init"}; 
  };

  StateUpdateEvent state_update_event_;

  /* Helper functions */

  // Print current state of FSM
  void print_current_state() {
    // TODO Perhaps use a std::map to solve this problem
    std::string current_state{""};

    if (PlannerFSM::is_in_state<INIT>()){
      current_state = "INIT";
    }
    else if (PlannerFSM::is_in_state<WAIT_TARGET>()){
      current_state = "WAIT_TARGET";
    }
    else if (PlannerFSM::is_in_state<SEQUENTIAL_START>()) {
      current_state = "SEQUENTIAL_START";
    }
    else {
      current_state = "UNDEFINED";
    }

    std::cout << "Current state: " << current_state << std::endl;
  }

private:

  void odometryCb(const nav_msgs::OdometryConstPtr &msg) {
    state_update_event_.have_odom = true;
  }

  void FSMUpdateTimerCb(const ros::TimerEvent &e) {
    TaskMaster::PlannerFSM::dispatch(state_update_event_);

    print_current_state();
  }

  /* Planning Methods */
  bool planFromGlobalTraj(const int trial_times) 
  {
    return true;
  }

  // Subscribers
  ros::Subscriber odom_sub_;

  // Timer to execute FSM Update callback
  ros::Timer fsm_update_timer_;

  // Parameters
  int drone_id{0};
};

FSM_INITIAL_STATE(TaskMaster::PlannerFSM, TaskMaster::INIT)

int main(int argc, char** argv) {

  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  TaskMaster task_mstr;
  
  task_mstr.init(nh);

  ros::spin();

  return 0;

}