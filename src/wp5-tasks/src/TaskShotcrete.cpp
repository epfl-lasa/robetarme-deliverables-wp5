#include "TaskShotcrete.h"

using namespace std;

TaskShotcrete::TaskShotcrete(ros::NodeHandle& n, double freq) : ITaskBase(n, freq){};

bool TaskShotcrete::initialize(std::string robotName) {
  cout << "TaskShotcrete::initialize() " << robotName << endl;
  return true;
}

bool TaskShotcrete::execute() {
  cout << "TaskShotcrete::execute()" << endl;
  return true;
}

bool TaskShotcrete::computePath() {
  cout << "TaskShotcrete::computePath()" << endl;
  return true;
}

bool TaskShotcrete::goHomingPosition() {
  cout << "TaskShotcrete::goHomingPosition()" << endl;
  return true;
}

bool TaskShotcrete::goWorkingPosition() {
  cout << "TaskShotcrete::goWorkingPosition()" << endl;
  return true;
}

void TaskShotcrete::setRoboticArm(vector<double> desiredJoint) { cout << "setRoboticArm()" << endl; }

void TaskShotcrete::setHomingPosition(vector<double> desiredJoint) { cout << "setHomingPosition()" << endl; }