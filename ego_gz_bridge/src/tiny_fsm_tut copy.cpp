#include "tinyfsm.hpp"
#include <iostream>

struct Off; 

// ----------------------------------------------------------------------------
// 1. Event Declarations
//

// This event is sent to the FSM on each callback to update the FSM
struct FSMUpdateEvent : tinyfsm::Event {
  bool have_target;
  bool have_trigger;
  bool have_odom;
  bool flag_escape_emergency_;

  bool have_recv_pre_agent; // All drones have assigned trajectories
};

// ----------------------------------------------------------------------------
// 2. State Machine Base Class Declaration
//
class EgoFSM : public tinyfsm::Fsm<EgoFSM>
{
public:
  virtual void react(FSMUpdateEvent const &) {};
  // alternative: enforce handling of Toggle in all states (pure virtual)
  // virtual void react(FSMUpdateEvent const &) = 0;

  virtual void entry(void) {};  /* entry actions in some states */
  void exit(void) {}; /* no exit actions*/

  int current_state = 0;
};

// ----------------------------------------------------------------------------
// Transition functions
//


// ----------------------------------------------------------------------------
// 4. State Declarations
//

class Init : public EgoFSM
{
  void entry() override {
    std::cout << "Entered INIT" << std::endl;
  }

  void react(FSMUpdateEvent const& event) override {
    if (!event.have_odom) {
      // goto force_return; // return;
    }
    else {
      transit<WAIT_TARGET>();
    }
  }

};

class WAIT_TARGET : public EgoFSM
{
  void entry() override {
    std::cout << "Entered WAIT_TARGET" << std::endl;
  }

  void react(FSMUpdateEvent const& event) override {
    if (!event.have_target || !event.have_trigger){
      // goto force_return; // return;
    }
    else {
      transit<SEQUENTIAL_START>();
    }
  }
};

class SEQUENTIAL_START : public EgoFSM
{
  void entry() override {
    std::cout << "Entered SEQUENTIAL_START" << std::endl;
  }

  void react(FSMUpdateEvent const& event) override {
    if (event.have_recv_pre_agent) {
      // Plan
      bool plan_success = true;
      if (plan_success) {
        transit<EXEC_TRAJ>();
      }
      else {
        transit<SEQUENTIAL_START>();
      }
    }
  }
};



// Declare initial state to be OFF
FSM_INITIAL_STATE(Switch, Off)

// ----------------------------------------------------------------------------
// 5. State machine list declaration (dispatches events to multiple FSM's)
//

// using fsm_list = tinyfsm::FsmList< Switch>
using fsm_handle = Switch;

class EgoPlanner {
public:
  

private:

};

int main (int argc, char** argv)
{
  // Instantiate events
  FSMUpdateEvent toggle;

  fsm_handle::start();

  while (1)
  {
    char c;
    std::cout << std::endl << "t=Toggle, q=Quit?" ;
    std::cin >> c;
    switch (c) {
    case 't':
      std::cout << "> Toggling switch..." << std::endl;
      fsm_handle::dispatch(fsm_update_event_);
      break;
    case 'q':
      return 0;
    default:
      std::cout << "> Invalid input" << std::endl;
    }
  }
}