// $CXX -std=c++14 euml_emulation.cpp
#include "sml.hpp"
#include <cassert>

namespace sml = boost::sml;

namespace {
struct FSMUpdate {
  bool have_odom{false};
  bool have_target{false};
  bool have_trigger{false};
  bool flag_escape_emergency{false};
  bool plan_success{false};

  int drone_id{0};
  bool have_recv_pre_agent{false}; // All drones have assigned trajectories
};
struct e2 {};
struct e3 {};

auto FSMUpdateEvent = sml::event<FSMUpdate>;
auto event2 = sml::event<e2>;
auto event3 = sml::event<e3>;

auto init = sml::state<class init>;
auto s1 = sml::state<class s1>;
auto s2 = sml::state<class s2>;

class euml_emulation;

struct Guard {
  template <class TEvent>
  bool operator()(euml_emulation&, const TEvent&) const;
} guard;

struct Action {
  template <class TEvent>
  void operator()(euml_emulation&, const TEvent&);
} action;

class euml_emulation {
 public:
  auto operator()() const {
    using namespace sml;
    return make_transition_table(
      s1 <= *init + FSMUpdateEvent,
      s2 <= s1    + event2 [ guard ],
      X  <= s2    + event3 [ guard ] / action
    );
  }

  template <class TEvent>
  bool call_guard(const TEvent&) {
    return true;
  }

  void call_action(const e3&) {}
};

template <class TEvent>
bool Guard::operator()(euml_emulation& sm, const TEvent& event) const {
  return sm.call_guard(event);
}

template <class TEvent>
void Action::operator()(euml_emulation& sm, const TEvent& event) {
  sm.call_action(event);
}
}  // namespace

int main() {
  euml_emulation euml;
  sml::sm<euml_emulation> sm{euml};
  assert(sm.is(init));
  // sm.process_event(FSMUpdate_event);
  // assert(sm.is(s1));
  // sm.process_event(e2{});
  // assert(sm.is(s2));
  // sm.process_event(e3{});
  // assert(sm.is(sml::X));
}