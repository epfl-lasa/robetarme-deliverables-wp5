#pragma once

// MSM back-end
#include <boost/msm/back/state_machine.hpp>

// MSM front-end
#include <boost/mp11/mpl.hpp>
#include <boost/msm/front/euml/euml.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include "ITaskBase.h"
#include "TaskFSM.h"

namespace msm = boost::msm;
namespace msmf = msm::front;
namespace mp11 = boost::mp11;

// List of FSM events
class Recover {};
class SafetyTrigger {};

// front-end: define the FSM structure
class TaskSafeFSM : public msmf::state_machine_def<TaskSafeFSM, TaskFSM> {
private:
  // List of FSM states
  class Running : public TaskFSM {};
  class SafetyIssue;

public:
  TaskSafeFSM(std::shared_ptr<ITaskBase> task){};

  typedef msm::back::state_machine<TaskFSM> Task;
  typedef Task initial_state;

  // Each row correspond to : Start, Event, Next, Action, Guard
  using transition_table = mp11::mp_list<
      // Task ----------------------------------------------
      msmf::Row<Task, SafetyTrigger, SafetyIssue, msmf::none, msmf::none>,

      // SafetyIssue ------------------------------------------
      msmf::Row<SafetyIssue, Recover, Task, msmf::none, msmf::none>>;
};

class TaskSafeFSM::SafetyIssue : public msmf::state<> {
public:
  template <class Event, class FSM>
  void on_entry(Event const& event, FSM& fsm) {
    std::cout << "Entering: TaskSafeFSM - SafetyIssue" << std::endl;
  }

  template <class Event, class FSM>
  void on_exit(Event const& event, FSM& fsm) {
    std::cout << "Leaving: TaskSafeFSM - SafetyIssue" << std::endl;
  }
};
