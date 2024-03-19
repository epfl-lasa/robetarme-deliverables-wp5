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
class TaskSafeFSM : public msmf::state_machine_def<TaskSafeFSM> {
private:
  // List of FSM states
  class SafetyIssue;

public:
  TaskSafeFSM(int test) : task(test) { std::cout << "Inside TaskSafeFSM : " << test << std::endl; };

  typedef task_fsm_ initial_state;

  using transition_table = mp11::mp_list<
      // Running ----------------------------------------------
      msmf::Row<task_fsm_, SafetyTrigger, SafetyIssue, msmf::none, msmf::none>,

      // SafetyIssue ------------------------------------------
      msmf::Row<SafetyIssue, Recover, task_fsm_, msmf::none, msmf::none>>;

private:
  task_fsm_ task;
};

typedef msm::back::state_machine<TaskSafeFSM> task_safe_fsm_;

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
