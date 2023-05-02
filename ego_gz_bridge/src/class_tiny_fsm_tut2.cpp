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
#include <chrono>
#include <thread>


namespace sml = boost::sml;

// ----------------------------------------------------------------------------
// 1. Event Declarations
//
// struct ack { 
//   bool valid{};
// };
// struct fin {
//   int id{};
//   bool valid{};
// };
// struct timeout {};
// struct release {};

struct FSMUpdate{
  bool have_odom{false};
  bool have_target{false};
  bool have_trigger{false};
  bool flag_escape_emergency{false};

  int drone_id{0};
  bool have_recv_pre_agent{false}; // All drones have assigned trajectories
};

// ----------------------------------------------------------------------------
// 2. Guards
// 
// auto is_ack_valid = [](const ack& ack_event) { 
//   if (ack_event.valid){
//     return true; 
//   }
//   else {
//     std::cout << "Invalid acknowledgement" << std::endl;
//     return false;
//   }
// };

// auto is_fin_valid = [](const fin& fin_event) { 
//   if (fin_event.valid){
//     return true; 
//   }
//   else {
//     std::cout << "Invalid fin" << std::endl;
//     return false;
//   }
// };

auto wait_guard = [](const FSMUpdate& event) { 
  if (event.have_odom){
    return true;
  }
  else {
    // goto force_return; // return;
    return false;
  }
};

auto start_guard = [](const FSMUpdate& event) { 
  if (event.have_target && event.have_trigger){
    return true;
  }
  else {
    // goto force_return; // return;
    return false;
  }
};

// ----------------------------------------------------------------------------
// 3. Actions
// 
// struct wait_guard {
//   void operator()() noexcept {
//     std::cout << "sent ack" << std::endl;
//   }
// };
// struct start_guard {
//   void operator()() noexcept {
//     std::cout << "sent fin" << std::endl;
//   }
// };

// ----------------------------------------------------------------------------
// 4. States
// 
class INIT;
class WAIT_TARGET;
class SEQUENTIAL_START;
class EXEC_TRAJ;

// ----------------------------------------------------------------------------
// 5. State machine
// 
struct hello_world {
  auto operator()() const {
    using namespace sml;
    // clang-format off
    return make_transition_table(
      // Initial state
      *state<INIT> + event<FSMUpdate> [ wait_guard ] = state<WAIT_TARGET>,

      state<WAIT_TARGET> + event<FSMUpdate> [ start_guard ] = state<SEQUENTIAL_START>,

      state<SEQUENTIAL_START> + event<FSMUpdate> [ plan_success ] = state<EXEC_TRAJ>

      // state<SEQUENTIAL_START> + event<FSMUpdate> [ start_guard ] = state<SEQUENTIAL_START>,
      
      // state<SEQUENTIAL_START> + event<fin> [ is_fin_valid ] / send_ack() = state<EXEC_TRAJ>,
      
      // state<EXEC_TRAJ> + event<timeout> / send_ack() = X
    );
  }
};

class FSMTop {
public:

  FSMTop(sml::sm<hello_world>& sm) : sm(sm) {
    std::cout << "FSMTop created!" << std::endl;
  }

  void plan_path() {
    std::cout << "Planning path!" << std::endl;
  }

  void init() {
    assert(sm.is(sml::state<INIT>));

    FSMUpdate fsm_update = FSMUpdate();

    fsm_update.have_odom = true;
    sm.process_event(fsm_update);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    assert(sm.is(sml::state<WAIT_TARGET>));

    fsm_update.have_target = true;
    fsm_update.have_trigger = true;
    sm.process_event(fsm_update);
    assert(sm.is(sml::state<SEQUENTIAL_START>));

    fsm_update.have_target = true;
    sm.process_event(fsm_update);
    assert(sm.is(sml::state<EXEC_TRAJ>));

    // sm.process_event(timeout{});
    // assert(sm.is(sml::X));  // released

    std::cout << "hello_world terminated!" << std::endl;
  }

private:
  sml::sm<hello_world>& sm;
};



int main() {
  using namespace sml;

  sm<hello_world> sm;
  FSMTop fsmtop(sm);

  fsmtop.init();
}