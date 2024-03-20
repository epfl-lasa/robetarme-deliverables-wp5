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
class Initialized {};
class PathComputed {};
class Start {};
class Finished {};

class ErrorTrigger {};
class ErrorAcknowledgement {};

// front-end: define the FSM structure
class TaskFSM : public msmf::state_machine_def<TaskFSM> {
private:
  // List of FSM states
  class Initializing;
  class Planning;

  class Ready : public msmf::state<> {
  public:
    enum ShareMode : uint8_t { NONE = 0, FOLLOWING, AUTOMATIC };
    ShareMode getShareMode() { return shareMode_; }

    void setShareMode(ShareMode share) { shareMode_ = share; }

    template <class Event, class FSM>
    void on_entry(Event const& event, FSM& fsm) {
      std::cout << "Entering: TaskFSM - Ready" << std::endl;
    }

    template <class Event, class FSM>
    void on_exit(Event const& event, FSM& fsm) {
      std::cout << "Leaving: TaskFSM - Ready" << std::endl;
    }

  private:
    ShareMode shareMode_ = ShareMode::NONE;
  };

  class HumanSharing;
  class Executing;
  class Homing;
  class Exit;

  class Safe;
  class AllOk;
  class ErrorMode;

protected:
  bool exit_ = true;
  bool homed_ = false;
  bool ready_ = false;

  std::string error_ = "";
  std::shared_ptr<ITaskBase> currentTask_;

public:
  TaskFSM(std::shared_ptr<ITaskBase> task) : currentTask_(task) { std::cout << "Inside TaskFSM : " << std::endl; };

  bool getExit() { return exit_; }
  bool getHomed() { return homed_; }
  bool getReady() { return ready_; }

  std::string getError() { return error_; }
  std::shared_ptr<ITaskBase> getCurrentTask() { return currentTask_; }

  void setExit(bool exit) { exit_ = exit; }
  void setHome(bool homed) { homed_ = homed; }
  void setReady(bool ready) { ready_ = ready; }

  void setError(std::string error) { error_ = error; }

  // Actions
  class goWorkingPosition {
  public:
    template <class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& evt, FSM& fsm, SourceState& src, TargetState& tgt) {
      bool feedback = fsm.getCurrentTask()->goWorkingPosition();

      if (feedback) {
        fsm.setReady(true);
      } else {
        fsm.setError("Error: Task goWorkingPosition failed");
        fsm.process_event(ErrorTrigger());
      }
    }
  };

  // Guard condition
  class isSharing {
  public:
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM& fsm, SourceState& src, TargetState& tgt) {
      return src.getShareMode() > TaskFSM::Ready::ShareMode::NONE;
    }
  };

  class isReady {
  public:
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM& fsm, SourceState& src, TargetState& tgt) {
      return fsm.getReady();
    }
  };

  class isHomed {
  public:
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM& fsm, SourceState& src, TargetState& tgt) {
      return fsm.getHomed();
    }
  };

  class isExiting {
  public:
    template <class EVT, class FSM, class SourceState, class TargetState>
    bool operator()(EVT const& evt, FSM& fsm, SourceState& src, TargetState& tgt) {
      return fsm.getExit();
    }
  };

  typedef mp11::mp_list<AllOk, Initializing> initial_state;

  // Each row correspond to : Start, Event, Next, Action, Guard
  using transition_table = mp11::mp_list<
      // Initializing -----------------------------------------
      msmf::Row<Initializing, Initialized, Planning, msmf::none, msmf::none>,

      // Planning ---------------------------------------------
      msmf::Row<Planning, PathComputed, Ready, goWorkingPosition, msmf::none>,

      // Ready ------------------------------------------------
      msmf::Row<Ready, Start, HumanSharing, msmf::none, msmf::euml::And_<isSharing, isReady>>,
      msmf::Row<Ready, Start, Executing, msmf::none, msmf::euml::And_<msmf::euml::Not_<isSharing>, isReady>>,

      // Executing --------------------------------------------
      msmf::Row<Executing, Finished, Ready, goWorkingPosition, msmf::euml::Not_<isExiting>>,
      msmf::Row<Executing, Finished, Homing, msmf::none, isExiting>,

      // Homing -----------------------------------------------
      msmf::Row<Homing, msmf::none, Exit, msmf::none, isHomed>,

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
    bool feedback = fsm.getCurrentTask()->initialize();

    if (feedback) {
      fsm.process_event(Initialized());
    } else {
      fsm.setError("Error: Task initialization failed");
      fsm.process_event(ErrorTrigger());
    }
  }
};

class TaskFSM::Planning : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    bool feedback = fsm.getCurrentTask()->computePath();

    if (feedback) {
      fsm.process_event(PathComputed());
    } else {
      fsm.setError("Error: Path computation failed");
      fsm.process_event(ErrorTrigger());
    }
  }
};

class TaskFSM::HumanSharing : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskFSM - HumanSharing" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskFSM - HumanSharing" << std::endl;
  }
};

class TaskFSM::Executing : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    bool feedback = fsm.getCurrentTask()->execute();

    if (feedback) {
      fsm.process_event(Finished());
    } else {
      fsm.setError("Error: Task execution failed");
      fsm.process_event(ErrorTrigger());
    }
  }
};

class TaskFSM::Homing : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    bool feedback = fsm.getCurrentTask()->goHomingPosition();

    if (!feedback) {
      fsm.setError("Error: Task homing failed");
      fsm.process_event(ErrorTrigger());
    }
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
