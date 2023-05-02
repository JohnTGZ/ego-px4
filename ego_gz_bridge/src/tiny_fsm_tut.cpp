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
  virtual void react(Toggle const &) {};
  // alternative: enforce handling of Toggle in all states (pure virtual)
  // virtual void react(Toggle const &) = 0;

  virtual void entry(void) {};  /* entry actions in some states */
  void exit(void) {}; /* no exit actions*/

  // Alternative: enforce entry actions in all states (pure virtual)
  // virtual void entry(void) = 0;

  int current_state = 0;
};

// ----------------------------------------------------------------------------
// Transition functions
//
void toggle_switch(bool toggle) {
  if (toggle) {
    std::cout << "*Switch is ON" << std::endl;
  }
  else {
    std::cout << "* Switch is OFF" << std::endl;
  }
}


// ----------------------------------------------------------------------------
// 4. State Declarations
//
class On : public Switch
{
  void entry() override {
    toggle_switch(true);
  }

  void react(Toggle const&) override {
    if (current_state == 1){
      std::cout << "Switch is already on, ignoring toggle" << std::endl;
      return;
    }
    transit<Off>();
  }
};

class Off : public Switch
{
  void entry() override {
    toggle_switch(false);
  }
  void react(Toggle const&) override {
    if (current_state == 0){
      std::cout << "Switch is already off, ignoring toggle" << std::endl;
      return;
    }
    transit<On>();
  }
};

// Declare initial state to be OFF
FSM_INITIAL_STATE(Switch, Off)

// ----------------------------------------------------------------------------
// 5. State machine list declaration (dispatches events to multiple FSM's)
//

// using fsm_list = tinyfsm::FsmList< Switch>
using fsm_handle = Switch;

int main (int argc, char** argv)
{
  // Instantiate events
  Toggle toggle;

  fsm_handle::start();

  while (1)
  {
    char c;
    std::cout << std::endl << "t=Toggle, q=Quit?" ;
    std::cin >> c;
    switch (c) {
    case 't':
      std::cout << "> Toggling switch..." << std::endl;
      fsm_handle::dispatch(toggle);
      break;
    case 'q':
      return 0;
    default:
      std::cout << "> Invalid input" << std::endl;
    }
  }
}