# Robetarme WP5 Deliverable

Welcome to the Robetarme WP5 Deliverable repository! This repository contains all the code necessary to create planners, dynamical systems, and controllers for shotcrete, surface finishing, and metal additive tasks. The codebase supports both torque and velocity control.

## Overview

The repository comprises multiple packages written in C++. Currently, it is designed to work with ROS Noetic, but plans are underway to transition it to ROS2.

## Installation

We do work on Ubuntu system using docker and its docker compose plugin, which is the environment we recommend. But if you want, you can have a look at our Dockerfile and the scripts in the root folder to have all the needed dependencies if you would like using an other way to install them.

### Dependencies

Assuming working on a Ubuntu Linux distribution, follow these instructions to have a working environment using docker and docker compose plugin V2.

- **docker** <https://docs.docker.com/engine/install/ubuntu/>

- **docker compose** <https://docs.docker.com/compose/install/linux/#install-using-the-repository>

#### Docker setup

We utilize Docker Compose for managing the Docker container. Follow these steps to set up the environment at the roots of the folder:

    # You need to defin a rosuser
    export ROS_USER=COMPUTERNAME

    # Initialize the submodules
    git submodule update --init --recursive
    git submodule update --recursive --remote

    # Build the Docker containers
    docker compose build

    # Start the Docker containers in detached mode
    docker compose up -d

    # Access the Docker container's shell
    # This will set up the necessary environment within Docker for running the codebase.
    docker exec -it robetarme-deliverables-wp5-ros-1 bash

    # Build code and source it
    catkin build
    source devel/setup.bash

## Documentation

The documentation has to be generated using the script inside doc folder. Please be sure to have the doxygen packet installed. On linux do :

    sudo apt install doxygen

The documentation is readable opening the index.html file into your preferred browser.

## Usage

Once the dependencies are installed, you can start using the codebase. Each package contains its own set of functionalities. Refer to the documentation within each package for detailed usage instructions.

To run a new task, use the launch file with the corresponding task, depending on the one to be done. If no task is specified, an error message will shows up and display the name of the taskType to be used. Here are some examples:

    roslaunch wp5_tasks main_task.launch taskType:=shotcrete
    roslaunch wp5_tasks main_task.launch taskType:=surface_finishing

## Packages

Here's a brief overview of the library included in this repository.

Each package contributes to different aspects of the robotic workflow, enabling comprehensive control and operation of robotic tasks. Refer to the documentation within each package for detailed usage instructions.

### Planner

Contains algorithms for path planning.

### Dynamical System

Implements dynamical systems for controlling robotic motions.

### Controller

Provides controllers for shotcrete, surface finishing, and metal additive tasks.

### ROS Interface

Handles communication with ROS, providing an interface for interacting with the robotic system.

### Safety

Implements safety mechanisms to ensure the safe operation of the robotic system.

### Shared Control

Provides shared control algorithms for collaborative human-robot interaction.

### Perception

Implements perception algorithms for environment sensing and object detection.

## Contributing

We welcome contributions to this repository. If you find any issues or have suggestions for improvements, please feel free to open an issue or submit a pull request.

## Contact

If you have any questions or need further assistance, please contact either :

- Tristan Bonato - <tristan_bonato@hotmail.com>
- Louis Munier - <lmunier@protonmail.com>

Thank you for using Robetarme WP5 Deliverable! We hope you find it useful in your robotic endeavors.

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Tristan Bonato - <tristan_bonato@hotmail.com>
- Rui Wu - <wurui19930213@gmail.com>
- Soheil Gholami - <gholamiisoheiil@gmail.com>
