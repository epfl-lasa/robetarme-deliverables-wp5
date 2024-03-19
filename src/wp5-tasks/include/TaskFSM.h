#pragma once

// MSM back-end
#include <boost/msm/back/state_machine.hpp>

// MSM front-end
#include <boost/mp11/mpl.hpp>
#include <boost/msm/front/euml/euml.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

// Other
#include <iostream>
#include <memory>
#include <string>

#include "ITaskBase.h"

namespace msm = boost::msm;
namespace msmf = msm::front;
namespace mp11 = boost::mp11;

// List of FSM events
class Start {};
class Initialized {};
class PathComputed {};
class Finished {};

class ErrorTrigger {};
class ErrorAcknowledgement {};

// front-end: define the FSM structure
class TaskFSM : public msmf::state_machine_def<TaskFSM> {
private:
  // List of FSM states
  class Initializing;
  class Planning;
  class Ready;
  class HumanShared;
  class AutoExecuting;
  class Homing;
  class Exit;

  class Safe;
  class AllOk;
  class ErrorMode;

protected:
  std::shared_ptr<ITaskBase> currentTask_;

public:
  TaskFSM(std::shared_ptr<ITaskBase> task) : currentTask_(task) { std::cout << "Inside TaskFSM : " << std::endl; };

  typedef mp11::mp_list<AllOk, Initializing> initial_state;

  // Each row correspond to : Start, Event, Next, Action, Guard
  using transition_table = mp11::mp_list<
      // Initializing -----------------------------------------
      msmf::Row<Initializing, Initialized, Planning, msmf::none, msmf::none>,

      // Planning ---------------------------------------------
      msmf::Row<Planning, PathComputed, Ready, msmf::none, msmf::none>,

      // Ready ------------------------------------------------
      msmf::Row<Ready, Start, AutoExecuting, msmf::none, msmf::none>,

      // AutoExecuting --------------------------------------------
      msmf::Row<AutoExecuting, Finished, Ready, msmf::none, msmf::none>,

      // Error ------------------------------------------------
      msmf::Row<AllOk, ErrorTrigger, ErrorMode, msmf::none, msmf::none>,
      msmf::Row<ErrorMode, ErrorAcknowledgement, AllOk, msmf::none, msmf::none>>;
};

typedef msm::back::state_machine<TaskFSM> taskFsm_;

// The list of FSM states
class TaskFSM::Initializing : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - Initializing" << std::endl;
    bool feedback = fsm.currentTask_->initialize("shotcrete");

    if (feedback) {
      fsm.process_event(Initialized());
    }
    std::cout << "Entering 2: TaskFSM - Initializing" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - Initializing" << std::endl;
  }
};

class TaskFSM::Planning : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - Planning" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - Planning" << std::endl;
  }
};

class TaskFSM::Ready : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - Ready" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - Ready" << std::endl;
  }
};

class TaskFSM::HumanShared : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - HumanShared" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - HumanShared" << std::endl;
  }
};

class TaskFSM::AutoExecuting : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - AutoExecuting" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - AutoExecuting" << std::endl;
  }
};

class TaskFSM::Homing : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - Homing" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - Homing" << std::endl;
  }
};

class TaskFSM::Exit : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - Exit" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - Exit" << std::endl;
  }
};

class TaskFSM::Safe : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - Safe" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - Safe" << std::endl;
  }
};

class TaskFSM::AllOk : public msm::front::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&) {
    std::cout << "starting: AllOk" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&) {
    std::cout << "finishing: AllOk" << std::endl;
  }
};

class TaskFSM::ErrorMode : public msm::front::interrupt_state<ErrorAcknowledgement> {
public:
  template <class Event, class FSM>
  void on_entry(Event const&, FSM&) {
    std::cout << "starting: ErrorMode" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const&, FSM&) {
    std::cout << "finishing: ErrorMode" << std::endl;
  }
};
