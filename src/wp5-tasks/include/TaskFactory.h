// clang off
#include "ITaskBase.h"
// clang on

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "TaskShotcrete.h"

class TaskFactory {
public:
  static std::shared_ptr<ITaskBase> createTask(const TaskType& taskType,
                                               ros::NodeHandle& nh,
                                               double freq,
                                               std::string robotName) {
    static std::map<TaskType, std::function<std::shared_ptr<ITaskBase>(ros::NodeHandle&, double, std::string)>>
        taskMap = {{TaskType::SHOTCRETE, [](ros::NodeHandle& nh, double freq, std::string robotName) {
                      return std::make_shared<TaskShotcrete>(nh, freq, robotName);
                    }}};

    auto it = taskMap.find(taskType);
    if (it != taskMap.end()) {
      try {
        std::shared_ptr<ITaskBase> taskBase =
            TaskFactory::createTask(ITaskBase::taskTypes.at(strTaskType), nh, rosFreq, robotName);
        std::shared_ptr<TaskShotcrete> taskShotcrete = std::dynamic_pointer_cast<TaskShotcrete>(taskBase);
        if (!taskShotcrete) {
          throw std::runtime_error("Invalid task type");
        }
      } catch (const std::out_of_range& e) {
        // Handle the exception
      }
      return it->second(nh, freq, robotName);
    } else {
      throw std::invalid_argument("Invalid task type");
    }
  }
};