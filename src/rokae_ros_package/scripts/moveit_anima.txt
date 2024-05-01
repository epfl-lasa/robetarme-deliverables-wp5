#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This script is designed to be used with Moveit and share to ANIMA to
# test simple trajectories on their Rokae CR7 robot.

# Author: Louis Munier
# Last update: 2024-04-17

import copy
import rospy
import signal
import sys
import tf
import threading
import time

from geometry_msgs.msg import Pose, PoseStamped
from math import pi
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, RobotCommander
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Bool


# Parameters to be set
# ======================== ANIMA : Please set the following parameters ========================
INIT_POSE = [0.082, -0.45, 0.65, pi / 2, 0, 0]
SCENARIO = 2  # 1 or 2
NB_ITERATIONS = 3  # Number of iterations of the path to be executed
STEP_BY_STEP = False  # If True, the robot will ask for enter to move to the next step
MAX_PATH_TRIAL = 10  # Maximum number of trials to generate a cartesian path

# Depending on the scenario we define the target offsets for the waypoints
# The target offsets are defined as dictionaries with the keys :
# - 'pos' [x, y, z]
# - 'angle' [roll, pitch, yaw]
# - 'speed'
# - 'weld'
# WARNING : Giving a rotation will not output in a straight line
if SCENARIO == 1:  # Multiples lines side by side with an offset
    LST_TARGET_OFFSET = [
        {'pos': [0, -0.05, 0], 'speed': 0.01, 'weld': False},
        {'pos': [-0.1, 0, 0], 'speed': 0.001, 'weld': True},
        {'pos': [0, 0.05, 0], 'speed': 0.05, 'weld': False},
        {'pos': [0.1, 0, -0.05], 'speed': 0.1, 'weld': False}
    ]
elif SCENARIO == 2:  # Multiples lines on top of each other
    LST_TARGET_OFFSET = [
        {'pos': [0, -0.05, 0], 'speed': 0.01, 'weld': False},
        {'pos': [-0.1, 0, 0], 'speed': 0.001, 'weld': True},
        {'pos': [0, 0.052, 0], 'speed': 0.05, 'weld': False},
        {'pos': [0.1, 0, 0], 'speed': 0.1, 'weld': False}
    ]
else:
    print("Invalid scenario.")
    exit(0)
# ======================== ANIMA : END of parameter region ========================

# Global flag
FLAG_STOP = False
FLAG_THREAD = False


def main():
    """
    Main function that initializes the ROS node, MoveIt, and executes the trajectory.
    """
    signal.signal(signal.SIGINT, signal_handler)

    try:
        scene, robot, move_group, pub_welding_state, pub_display_trajectory = init_moveit()
        get_information(robot, move_group)
        add_obstacles(scene, robot_platform=True, welding_boxe=False)

        init_pose = generate_init_pose()
        move_group.set_pose_target(init_pose)
        plan_init_pose = move_group.plan()

        display_trajectory(robot, plan_init_pose[1], pub_display_trajectory)
        input("============ Press `Enter` to move to the initial pose ... ============")

        move_group.execute(plan_init_pose[1], wait=True)
        move_group.stop()

        plan_execute_trajectory(
            move_group, robot, pub_welding_state, pub_display_trajectory)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


def signal_handler(sig, frame):
    """
    Signal handler function that handles the Ctrl+C signal.
    """
    global FLAG_STOP, FLAG_THREAD

    print('You pressed Ctrl+C!')

    if FLAG_THREAD:
        FLAG_STOP = True
    else:
        sys.exit(0)


def add_obstacles(scene, **kwargs):
    """
    Function to add obstacles to the planning scene.

    Args:
        scene: The PlanningSceneInterface object.
        kwargs: Additional keyword arguments.
            robot_platform: If True, add the robot platform obstacle.
            welding_boxe: If True, add the welding box obstacle.
    """
    if kwargs.get('robot_platform', False):
        scene.add_box(
            "robot_platform",
            generate_obstacle(
                [-0.3, 0.0, -0.25, 0.0, 0.0, 0.0, 1.0]
            ),
            size=(1.3, 0.77, 0.54)
        )

    if kwargs.get('welding_boxe', False):
        scene.add_box(
            "welding_boxe",
            generate_obstacle(
                [-0.2, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]
            ),
            size=(0.2, 0.77, 0.6)
        )


def plan_execute_trajectory(move_group, robot, pub_welding_state, pub_display_trajectory):
    """
    Function to plan and execute the trajectory.

    Args:
        move_group: The MoveGroupCommander object.
        robot: The RobotCommander object.
        pub_welding_state: The publisher for the welding state.
        pub_display_trajectory: The publisher for the display trajectory.
    """
    global FLAG_THREAD
    waypoints = []
    waypoints.append(move_group.get_current_pose().pose)

    for i in range(NB_ITERATIONS):
        for to in LST_TARGET_OFFSET:
            waypoints = generate_waypoints(waypoints[-1], [to])

            if i == NB_ITERATIONS - 1 and to == LST_TARGET_OFFSET[-1]:
                break

            # Compute the Cartesian path
            iter = 0
            fraction = 0.0
            while fraction < 1.0:
                iter += 1

                if iter > MAX_PATH_TRIAL:
                    print(
                        f"Path planning was not successful, fraction: {fraction}, iteration {i}.")
                    return

                (plan_trajectory, fraction) = move_group.compute_cartesian_path(
                    waypoints,   # waypoints to follow
                    0.01,        # eef_step
                    0.0,         # jump_threshold
                    True)        # avoid_collisions

                print("Fraction of cartesian path well computed: ", fraction)

            # Scale the velocity of the trajectory
            plan_trajectory = move_group.retime_trajectory(
                robot.get_current_state(), plan_trajectory, to['speed']
            )

            # Display the trajectory
            display_trajectory(robot, plan_trajectory, pub_display_trajectory)

            if STEP_BY_STEP:
                input(
                    "============ Press `Enter` to execute the trajectory ... ============")

            # Publish the welding state
            pub_welding_state.publish(to['weld'])

            # Execute the plan in a separate thread
            execute_thread = threading.Thread(
                target=move_group.execute, args=(plan_trajectory, True))
            execute_thread.start()

            # Wait for the execution to finish or for the stop flag to be set
            while execute_thread.is_alive():
                FLAG_THREAD = True

                if FLAG_STOP:
                    print('Stopping...')
                    move_group.stop()
                    execute_thread.join()
                    return

                time.sleep(0.1)

            FLAG_THREAD = False

    move_group.stop()


def init_moveit() -> tuple:
    """
    Function to initialize MoveIt and the ROS node.

    Returns:
        A tuple containing the scene, robot, move_group, pub_welding_state, and pub_display_trajectory objects.
    """
    roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    robot = RobotCommander()
    scene = PlanningSceneInterface()

    group = MoveGroupCommander("rokae_arm")
    group.set_pose_reference_frame("world")

    pub_welding_state = rospy.Publisher("welding_state", Bool, queue_size=1)
    pub_display_trajectory = rospy.Publisher(
        "/move_group/display_planned_path",
        DisplayTrajectory,
        queue_size=20,
    )

    # Wait for the scene to get ready
    rospy.sleep(1)

    return scene, robot, group, pub_welding_state, pub_display_trajectory


def get_information(robot, move_group):
    """
    Function to get information about the robot and the MoveGroup.

    Args:
        robot: The RobotCommander object.
        move_group: The MoveGroupCommander object.
    """
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print(f"============ Reference frame: {planning_frame}")

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print(f"============ End effector: {eef_link}")

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print(f"============ Robot Groups: {group_names}")

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state:")
    print(robot.get_current_state())
    print("")


def is_valid_quaternion(q):
    """
    Function to check if a quaternion is valid.

    Args:
        q: The quaternion to check.

    Returns:
        True if the quaternion is valid, False otherwise.
    """
    return abs(q.x**2 + q.y**2 + q.z**2 + q.w**2 - 1.0) < 1e-6


def generate_init_pose() -> Pose:
    """
    Function to generate the initial pose of the robot.

    Returns:
        The initial pose as a Pose object.
    """
    # Define the initial pose of the robot
    init_pose = Pose()
    init_pose.position.x = INIT_POSE[0]
    init_pose.position.y = INIT_POSE[1]
    init_pose.position.z = INIT_POSE[2]

    # Orientation
    roll = INIT_POSE[3]
    pitch = INIT_POSE[4]
    yaw = INIT_POSE[5]
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    init_pose.orientation.x = q[0]
    init_pose.orientation.y = q[1]
    init_pose.orientation.z = q[2]
    init_pose.orientation.w = q[3]

    if not is_valid_quaternion(init_pose.orientation):
        print("Invalid quaternion.")
        exit(0)

    return init_pose


def generate_obstacle(pose: list) -> PoseStamped:
    """
    Function to generate an obstacle pose.

    Args:
        pose: The pose of the obstacle as a list.

    Returns:
        The obstacle pose as a PoseStamped object.
    """
    # Define the pose of a box (in the same frame as the robot)
    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"

    box_pose.pose.position.x = pose[0]
    box_pose.pose.position.y = pose[1]
    box_pose.pose.position.z = pose[2]

    box_pose.pose.orientation.x = pose[3]
    box_pose.pose.orientation.y = pose[4]
    box_pose.pose.orientation.z = pose[5]
    box_pose.pose.orientation.w = pose[6]

    return box_pose


def generate_waypoints(init_pose: Pose, target_offset: list) -> list:
    """
    Function to generate the waypoints for the trajectory.

    Args:
        init_pose: The initial pose of the robot.
        target_offset: The target offset for the waypoints.

    Returns:
        The waypoints as a list of Pose objects.
    """
    waypoints = []
    new_pose = copy.deepcopy(init_pose)

    # Create the waypoints by incrementing x, z, and y positions or roll, pitch,
    # yaw angle respectively
    for target in target_offset:
        for k, v in target.items():
            if k == 'pos':
                new_pose.position.x = new_pose.position.x + v[0]
                new_pose.position.y = new_pose.position.y + v[1]
                new_pose.position.z = new_pose.position.z + v[2]

            if k == 'angle':
                q_trans = tf.transformations.quaternion_from_euler(
                    v[0], v[1], v[2]
                )

                q_orig = [
                    new_pose.orientation.x, new_pose.orientation.y,
                    new_pose.orientation.z, new_pose.orientation.w
                ]

                quat_new = tf.transformations.quaternion_multiply(
                    q_orig, q_trans
                )

                new_pose.orientation.x = quat_new[0]
                new_pose.orientation.y = quat_new[1]
                new_pose.orientation.z = quat_new[2]
                new_pose.orientation.w = quat_new[3]

        waypoints.append(copy.deepcopy(new_pose))

    return waypoints


def display_trajectory(robot, plan, pub_display_trajectory):
    """
    Function to display the planned trajectory.

    Args:
        robot: The RobotCommander object.
        plan: The planned trajectory.
        pub_display_trajectory: The publisher for the display trajectory.
    """
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)

    pub_display_trajectory.publish(display_trajectory)


if __name__ == '__main__':
    main()
