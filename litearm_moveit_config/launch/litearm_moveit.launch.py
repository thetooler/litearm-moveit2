#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_moveit.launch.py — the MoveIt planning stack for the litearm arm.

Starts, in any combination:

    1. the control stack   litearm_ros2_control/litearm_control.launch.py
                           (ros2_control_node + the direct-USB hardware component
                           + controller_manager + JSB + JTC)
    2. move_group          MoveIt planning (OMPL + KDL)
    3. RViz                the MoveIt MotionPlanning panel

All three start by default. When the control stack already runs somewhere else
(for example a bring-up shell you started by hand), pass start_control:=false so
that two controller_managers do not fight over the same command interfaces.

    # real arm (board powered, USB connected, licence activated)
    ros2 launch litearm_moveit_config litearm_moveit.launch.py

    # a different USB device
    ros2 launch litearm_moveit_config litearm_moveit.launch.py port:=/dev/ttyACM1

    # let a latched fault stop the launch instead of clearing it
    ros2 launch litearm_moveit_config litearm_moveit.launch.py clear_faults:=false

The control stack clears a latched fault on the arm while configuring it (clear_faults,
on by default). The firmware latches an EMERGENCY or a joint_fault and then refuses
ENABLE until a reset, which would otherwise make every bring-up after a fault a two-step
operation. Turn it off when you want the fault to stop the launch so you can look at the
arm first.

    # MoveIt only, control stack already running
    ros2 launch litearm_moveit_config litearm_moveit.launch.py start_control:=false

Feed-forward is the firmware's business: the PD gains and the gravity, friction,
integral and kd_extra terms live in the litearm-stm32 firmware, whose model is
generated from the URDF and compiled in. The MOVE_JS channel this stack uses has
no M·q̈ or C·q̇ term, because MOVE_JS has no acceleration source.

Unlike the shared-memory deployment, this one has no dry-run mode: the direct-USB
component talks to the real board, so a launch either has an arm on the other end
of the USB cable or its hardware component fails to configure.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

DEFAULT_DOMAIN_ID = "42"
"""Deliberately not 0: domain 0 is the one other equipment on the network uses."""


def _parse_bool(value, fallback=False):
    if value is None:
        return fallback
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return fallback


def _isolation_env(domain_id, localhost_only):
    """The environment every node of this stack gets: fixed domain, localhost only."""
    return {
        "ROS_DOMAIN_ID": str(domain_id).strip() or DEFAULT_DOMAIN_ID,
        "ROS_LOCALHOST_ONLY": "1" if _parse_bool(localhost_only, True) else "0",
    }


def _isolation_hint(domain_id, localhost_only):
    domain = str(domain_id).strip() or DEFAULT_DOMAIN_ID
    if _parse_bool(localhost_only, True):
        return f"export ROS_DOMAIN_ID={domain}; export ROS_LOCALHOST_ONLY=1"
    return f"export ROS_DOMAIN_ID={domain}    (localhost-only is off: expect cross-machine traffic)"


def _is_true(value):
    return _parse_bool(value, False)


def _declare_arguments():
    return [
        DeclareLaunchArgument("start_control", default_value="true",
                              description="Also start the ros2_control stack"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Start RViz2 with the MoveIt MotionPlanning panel"),
        DeclareLaunchArgument("port", default_value="",
                              description="Passed to the control stack: USB CDC device "
                                          "path; empty = auto-discovery (1d50:606f)"),
        DeclareLaunchArgument("clear_faults", default_value="true",
                              description="Passed to the control stack: clear a latched "
                                          "arm fault (EMERGENCY / joint_fault) before the "
                                          "hardware is enabled, so a bring-up after a fault "
                                          "stays a single command; false lets the fault stop "
                                          "the launch instead"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="move_group log level"),
        DeclareLaunchArgument("ros_domain_id", default_value=DEFAULT_DOMAIN_ID,
                              description="ROS domain of this stack (deliberately not 0); "
                                          "set 0 to integrate with another system"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = localhost discovery only: a second robot "
                                          "on the network otherwise shows up in RViz and "
                                          "as a second /move_action server"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    start_control = _is_true(resolve("start_control"))
    use_rviz = _is_true(resolve("use_rviz"))
    port = resolve("port").strip()
    domain_id = resolve("ros_domain_id").strip() or DEFAULT_DOMAIN_ID
    localhost_only = resolve("ros_localhost_only")
    env = _isolation_env(domain_id, localhost_only)

    moveit_share = get_package_share_directory("litearm_moveit_config")
    control_share = get_package_share_directory("litearm_ros2_control")

    # The MoveIt side expands the same URDF the control stack does, so the port
    # argument has to reach both; a mismatch would not error, it would just leave
    # move_group describing a robot the controller_manager is not running.
    moveit_config = (
        MoveItConfigsBuilder("litearm", package_name="litearm_moveit_config")
        .robot_description(mappings={"litearm_port": port})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    actions = []

    if start_control:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_share, "launch", "litearm_control.launch.py")),
            launch_arguments={
                "port": port,
                "clear_faults": "true" if _is_true(resolve("clear_faults")) else "false",
                # The isolation arguments have to be forwarded: the included nodes
                # land in their own environment, and move_group only sees the
                # controller_manager actions when both sides share a domain.
                "ros_domain_id": domain_id,
                "ros_localhost_only": localhost_only,
            }.items(),
        ))

    actions.append(Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        additional_env=env,
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", resolve("log_level")],
    ))

    if use_rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(moveit_share, "rviz", "moveit.rviz")],
            output="log",
            additional_env=env,
            # RViz's MoveIt plugin needs these parameter groups itself; move_group's
            # copy does not reach it.
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ))

    actions.insert(0, LogInfo(msg=(
        "──────── litearm MoveIt ────────\n"
        f"  control stack: {'started here' if start_control else 'expected to be running'}\n"
        f"  port: {port or '(auto-discovery, VID:PID 1d50:606f)'}\n"
        "  ⚠ This launch fixes the ROS domain; to use ros2 commands or the RViz\n"
        "    GUI from your own terminal, run:\n"
        f"        {_isolation_hint(domain_id, localhost_only)}\n"
        "────────────────────────────────")))
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
