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

namespace sml = boost::sml;

// ----------------------------------------------------------------------------
// 1. Event Declarations
//
struct ack { 
  bool valid{};
};
struct fin {
  int id{};
  bool valid{};
};
struct timeout {};
struct release {};

// ----------------------------------------------------------------------------
// 2. Guards
// 
auto is_ack_valid = [](const ack& ack_event) { 
  if (ack_event.valid){
    return true; 
  }
  else {
    std::cout << "Invalid acknowledgement" << std::endl;
    return false;
  }
};

auto is_fin_valid = [](const fin& fin_event) { 
  if (fin_event.valid){
    return true; 
  }
  else {
    std::cout << "Invalid fin" << std::endl;
    return false;
  }
};

// ----------------------------------------------------------------------------
// 3. Actions
// 
struct send_ack {
  void operator()() noexcept {
    std::cout << "sent ack" << std::endl;
  }
};
struct send_fin {
  void operator()() noexcept {
    std::cout << "sent fin" << std::endl;
  }
};

// ----------------------------------------------------------------------------
// 4. States
// 
class established;
class fin_wait_1;
class fin_wait_2;
class timed_wait;

// ----------------------------------------------------------------------------
// 5. State machine
// 
struct hello_world {
  auto operator()() const {
    using namespace sml;
    // clang-format off
    return make_transition_table(
      *state<established> + event<release> / send_fin() = state<fin_wait_1>,
      state<fin_wait_1> + event<ack> [ is_ack_valid ] = state<fin_wait_2>,
      state<fin_wait_2> + event<fin> [ is_fin_valid ] / send_ack() = state<timed_wait>,
      state<timed_wait> + event<timeout> / send_ack() = X
    );
  }
};

class FSMTop {
public:
  FSMTop(sml::sm<hello_world>& sm) : sm(sm) {
    std::cout << "FSMTop created!" << std::endl;
  }

  void init() {
    assert(sm.is(sml::state<established>));

    sm.process_event(release{});
    assert(sm.is(sml::state<fin_wait_1>));

    sm.process_event(ack{true});
    assert(sm.is(sml::state<fin_wait_2>));

    sm.process_event(fin{42,true});
    assert(sm.is(sml::state<timed_wait>));

    sm.process_event(timeout{});
    assert(sm.is(sml::X));  // released

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