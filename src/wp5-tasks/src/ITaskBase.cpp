// clang off
#include "ITaskBase.h"
// clang on

using namespace std;
using namespace Eigen;

ITaskBase::ITaskBase(ros::NodeHandle& nh, double freq) : nh_(nh), rosFreq_(freq), loopRate_(freq) {}
