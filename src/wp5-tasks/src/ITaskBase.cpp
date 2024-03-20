// clang off
#include "ITaskBase.h"
// clang on

using namespace std;
using namespace Eigen;

ITaskBase::ITaskBase(ros::NodeHandle& nh, double freq, string robotName) : nh_(nh), rosFreq_(freq), loopRate_(freq) {
  // Create an unique pointer for the instance of RosInterfaceNoetic
  rosInterface_ = make_unique<RosInterfaceNoetic>(nh_, robotName);
}
