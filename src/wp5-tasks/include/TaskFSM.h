#pragma once

// MSM back-end
#include <boost/msm/back/state_machine.hpp>

// MSM front-end
#include <boost/mp11/mpl.hpp>
#include <boost/msm/front/euml/euml.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

namespace msm = boost::msm;
namespace msmf = msm::front;
namespace mp11 = boost::mp11;

// List of FSM events
class Run {};
class Initialized {};
class PathComputed {};
class Finished {};

// front-end: define the FSM structure
class TaskFSM : public msmf::state_machine_def<TaskFSM> {
private:
  // List of FSM states
  class Initializing;
  class ComputingPath;
  class Ready;
  class Running;

public:
  TaskFSM(){};

  typedef Initializing initial_state;

  // Each row correspond to : Start, Event, Next, Action, Guard
  using transition_table = mp11::mp_list<
      // Initializing --------------------------------------
      msmf::Row<Initializing, Initialized, ComputingPath, msmf::none, msmf::none>,

      // ComputingPath -------------------------------------
      msmf::Row<ComputingPath, PathComputed, Ready, msmf::none, msmf::none>,

      // Ready ---------------------------------------------
      msmf::Row<Ready, Run, Running, msmf::none, msmf::none>,

      // Running -------------------------------------------
      msmf::Row<Running, Finished, Ready, msmf::none, msmf::none>>;
};

// The list of FSM states
class TaskFSM::Initializing : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: Initializing" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: Initializing" << std::endl;
  }
};

class TaskFSM::ComputingPath : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: ComputingPath" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: ComputingPath" << std::endl;
  }
};

class TaskFSM::Ready : public msmf::state<> {
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

class TaskFSM::Running : public msmf::state<> {
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
