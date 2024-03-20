# Robetarme WP5 Deliverable

Welcome to the Robetarme WP5 Deliverable repository! This repository contains all the code necessary to create planners, dynamical systems, and controllers for shotcrete, surface finishing, and metal additive tasks. The codebase supports both torque and velocity control.
## Overview

The repository comprises multiple packages written in C++. Currently, it is designed to work with ROS Noetic, but plans are underway to transition it to ROS2.
## Installation

To install the dependencies, you have two options:

Using Docker Compose:
We utilize Docker Compose for managing the Docker container. Follow these steps to set up the environment at the roots of the folder:

    # Build the Docker containers
    docker-compose build

    # Start the Docker containers in detached mode
    docker-compose up -d

    # Access the Docker container's shell
    docker exec -it robetarme-deliverables-wp5-ros-1 bash


This will set up the necessary environment within Docker for running the codebase.

Manual Installation:
If you prefer to install the dependencies directly onto your machine, you can use the provided scripts. Navigate to the scripts directory and run the appropriate script for your operating systemChoose the method that best fits your needs and environment to get started with the Robetarme WP5 Deliverable repository.
## Usage

There are several ways you can utilize the packages within this repository:

- Static Library Integration:

You can directly use the packages as a static library in your codebase. Simply include the necessary headers and link against the compiled library.

- Convenience Functions:

We've prepared a set of convenience functions to streamline the integration of tasks into your projects. These functions are designed to simplify common operations and enhance ease of use. Some examples include:

- Initialize Task: Initializes the task environment, setting up necessary parameters and configurations.
- Homing: Moves the robotic system to a predefined home position for safe starting or ending of tasks.
- DoTask: Executes the specified task, handling all necessary actions and interactions.
- GoPosition: Navigates the robotic system to a designated position within the workspace.

These functions are well-documented using Doxygen. You can find the documentation here.

- Finite State Machine (FSM):

For the shotcrete task, we've implemented a Finite State Machine (FSM) that manages all the functions for you. This simplifies the task execution process and provides an organized workflow. Currently, the FSM is under development for the metal additive and surface finishing tasks.

- Special Docker for Triggering FSM:

Additionally, we provide a special Docker container designed to run an action and wait for specific commands to trigger the Finite State Machine (FSM). This container streamlines the execution process and facilitates seamless integration with the FSM functionality.

Choose the method that best fits your project requirements and workflow. Don't hesitate to refer to the documentation for detailed information on each function and module.## Packages

Here's a brief overview of the library included in this repository:

- Planner: Contains algorithms for path planning.
- Dynamical System: Implements dynamical systems for controlling robotic motions.
- Controller: Provides controllers for shotcrete, surface finishing, and metal additive tasks.
- ROS Interface: Handles communication with ROS, providing an interface for interacting with the robotic system.
- Safety: Implements safety mechanisms to ensure the safe operation of the robotic system.
- Shared Control: Provides shared control algorithms for collaborative human-robot interaction.
- Perception: Implements perception algorithms for environment sensing and object detection.

Each package contributes to different aspects of the robotic workflow, enabling comprehensive control and operation of robotic tasks. Refer to the documentation within each package for detailed usage instructions.

## Contributing

We welcome contributions to this repository. If you find any issues or have suggestions for improvements, please feel free to open an issue or submit a pull request.
## License

This project is licensed under the MIT License.
##Contact

If you have any questions or need further assistance, please contact tristan_bonato@hotmail.com.

Thank you for using Robetarme WP5 Deliverable! We hope you find it useful in your robotic endeavors.

## Maintainers

- Louis Munier - <lmunier@protonmail.com>
- Tristan Bonato - <tristan_bonato@hotmail.com>
- Rui Wu - <wurui19930213@gmail.com>
- Soheil Gholami - <gholamiisoheiil@gmail.com>
