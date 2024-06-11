/**
 * @file ToolsSurfaceFinishing.cpp
 * @author Louis Munier (lmunier@protonmail.com)
 * @author Tristan Bonato (tristan_bonato@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-27
 *
 * @copyright Copyright (c) 2024 - EPFL
 *
 */
#include "ToolsSurfaceFinishing.h"

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <fstream>

#include "ToolsSurfaceFinishing.h"

using namespace std;

ToolsSurfaceFinishing::ToolsSurfaceFinishing(ros::NodeHandle& n) : IToolsBase(n) { takeYaml("surface_finishing"); }
