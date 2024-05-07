# Robetarme WP5 Deliverable

Welcome to the Robetarme WP5 Deliverable repository! This repository contains all the code necessary to create planners, dynamical systems, and controllers for shotcrete, surface finishing, and metal additive tasks. The codebase supports both torque and velocity control.

## Overview

The repository comprises multiple packages written in C++. Currently, it is designed to work with ROS Noetic, but plans are underway to transition it to ROS2.

## Installation

To install the dependencies, you have two options:

    Using Docker Compose:

    We utilize Docker Compose for managing the Docker container. Follow these steps to set up the environment at the roots of the folder:

    #you need to defin a rosuser
    export ROS_USER=COMPUTERNAME

    # initialize the submodules
    git submodule update --init --recursive
    git submodule update --recursive --remote

    # Build the Docker containers
    docker compose build

    # Start the Docker containers in detached mode
    docker compose up -d

    # Access the Docker container's shell
    docker exec -it robetarme-deliverables-wp5-ros-1 bash

    This will set up the necessary environment within Docker for running the codebase.

    Manual Installation:

    If you prefer to install the dependencies directly onto your machine, you can use the provided scripts. Navigate to the scripts directory and run the appropriate script for your operating system.

Choose the method that best fits your needs and environment to get started with the Robetarme WP5 Deliverable repository.

## Usage

Once the dependencies are installed, you can start using the codebase. Each package contains its own set of functionalities. Refer to the documentation within each package for detailed usage instructions.

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
