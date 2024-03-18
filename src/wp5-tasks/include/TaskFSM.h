#pragma once

// MSM back-end
#include <boost/msm/back/state_machine.hpp>

// MSM front-end
#include <boost/mp11/mpl.hpp>
#include <boost/msm/front/euml/euml.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

namespace msm = boost::msm;
namespace mp11 = boost::mp11;

using namespace std;
using namespace msm::front;

// List of FSM events
class Start {};
class Stop {};
class Initialized {};
class Finished {};

// front-end: define the FSM structure
class TaskFSM : public msm::front::state_machine_def<TaskFSM> {
private:
  // List of FSM states
  class Idle;
  class Initializing;
  class Ready;
  class Running;
  class Stopped;

  bool checkFSMInitialization_ = false;

public:
  TaskFSM(){};

  bool getCheckFSMInitialization() { return checkFSMInitialization_; }

  void setCheckFSMInitialization(bool checkInit) { checkFSMInitialization_ = checkInit; }

  // Guard condition
  class isInitialized {

  public:
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM& fsm, SourceState&, TargetState&) {
      // return true if the system is initialized, false otherwise
      return fsm.getCheckFSMInitialization();
    }
  };

  typedef Idle initial_state;

  // Each row correspond to : Start, Event, Next, Action, Guard
  using transition_table = mp11::mp_list<
      // Idle ----------------------------------------------
      Row<Idle, Start, Initializing, none, euml::Not_<isInitialized>>,
      Row<Idle, Start, Running, none, isInitialized>,
      Row<Idle, Stop, Stopped, none, none>,

      // Initializing --------------------------------------
      Row<Initializing, Initialized, Ready, none, none>,
      Row<Initializing, Stop, Stopped, none, none>,

      // Ready ---------------------------------------------
      Row<Ready, Start, Running, none, none>,
      Row<Ready, Stop, Stopped, none, none>,

      // Running -------------------------------------------
      Row<Running, Finished, Idle, none, none>,
      Row<Running, Stop, Stopped, none, none>>;
};

// The list of FSM states
class TaskFSM::Idle : public msm::front::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: Idle" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: Idle" << std::endl;
  }
};

class TaskFSM::Initializing : public msm::front::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: Initializing" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    fsm.setCheckFSMInitialization(true);
    std::cout << "Leaving: Initializing" << std::endl;
  }
};

class TaskFSM::Ready : public msm::front::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: Ready" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: Ready" << std::endl;
  }
};

class TaskFSM::Running : public msm::front::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: Running" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: Running" << std::endl;
  }
};

class TaskFSM::Stopped : public msm::front::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: Stopped" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: Stopped" << std::endl;
  }
};