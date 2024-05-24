/**
 * @file IToolsBase.h
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-03-07
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#include "IToolsBase.h"
#include "fstream"


#include <OsqpEigen/OsqpEigen.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;

void IToolsBase::takeYaml(string taskname) {
  taskname_ = taskname;
  string alternativeYamlPath = string(WP5_TOOLS_DIR) + "/config/tools_config.yaml";
  string yamlPath = string(WP5_TOOLS_DIR) + "/../../config/tools_config.yaml";

  // Check if the alternative YAML file exists
  ifstream originalFile(yamlPath);
  if (originalFile.good()) {
    cout << "Using general YAML file: " << yamlPath << endl;
  } else {
    yamlPath = alternativeYamlPath;
    cout << "Using local YAML file: " << yamlPath << endl;
  }

  // Load parameters from YAML file
  YAML::Node config = YAML::LoadFile(yamlPath);
  YAML::Node task = config[taskname];

  // Access parameters from the YAML file
  offsetTool_ = task["offsetTool"].as<double>();
  offsetTarget_ = task["offsetTarget"].as<double>();
  offsetTotal_ = offsetTool_ + offsetTarget_;
}

double IToolsBase::getOffset() { return offsetTool_; }
