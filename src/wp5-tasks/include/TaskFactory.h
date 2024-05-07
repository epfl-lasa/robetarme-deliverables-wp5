// clang off
#include "ITaskBase.h"
// clang on

#include <ros/ros.h>

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "TaskShotcrete.h"
#include "TaskSurfaceFinishing.h"

class TaskFactory {
public:
  using FactoryFunction =
      std::function<std::unique_ptr<ITaskBase>(ros::NodeHandle& nh, double freq, std::string robotName)>;

  TaskFactory() {
    registerTask("shotcrete", [](ros::NodeHandle& nh, double freq, std::string robotName) {
      return std::make_unique<TaskShotcrete>(nh, freq, robotName);
    });
    registerTask("surface_finishing", [](ros::NodeHandle& nh, double freq, std::string robotName) {
      return std::make_unique<TaskSurfaceFinishing>(nh, freq, robotName);
    });
  }

  void registerTask(std::string name, FactoryFunction function) { factoryFunctionRegistry[name] = function; }

  std::unique_ptr<ITaskBase> createTask(std::string name, ros::NodeHandle& nh, double freq, std::string robotName) {
    auto it = factoryFunctionRegistry.find(name);

    if (it != factoryFunctionRegistry.end()) {
      return it->second(nh, freq, robotName);
    } else {
      throw std::runtime_error("Invalid name");
    }
  }

  std::vector<std::string> getTaskTypes() {
    std::vector<std::string> keys;

    for (const auto& pair : factoryFunctionRegistry) {
      keys.push_back(pair.first);
    }

    return keys;
  }

private:
  std::map<std::string, FactoryFunction> factoryFunctionRegistry;
};