#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""demo.launch.py — MoveIt planning demo, without a control stack.

Brings up move_group and RViz with the arm description so you can drag a target
state in the MotionPlanning panel and press **Plan**. Execution is not available
here: this deployment's hardware component talks to a real board over USB, so
there is nothing to execute against without an arm.

    ros2 launch litearm_moveit_config demo.launch.py

A separate entry point from litearm_moveit.launch.py on purpose: "demo" and "real
arm" are two commands, so a wrong argument cannot bring a physical arm up by
accident.

Real robot (board powered, USB connected, licence activated, arm supported and
the emergency stop within reach):

    ros2 launch litearm_moveit_config litearm_moveit.launch.py

Named states available in RViz (the SRDF group_state):

    zero     all joints at zero
    ready    elbows up, a better planning start than the near-singular zero pose

Without a display, add use_rviz:=false and verify from the command line.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _declare_arguments():
    return [
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Start RViz2 with the MoveIt panel"),
        DeclareLaunchArgument("ros_domain_id", default_value="42",
                              description="ROS domain of this demo (deliberately not 0)"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = localhost discovery only"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("litearm_moveit_config") +
            "/launch/litearm_moveit.launch.py"),
        launch_arguments={
            # Hard-wired: this entry point never starts the control stack, so it
            # never opens the USB port and never moves a real arm.
            "start_control": "false",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "ros_domain_id": resolve("ros_domain_id"),
            "ros_localhost_only": resolve("ros_localhost_only"),
        }.items())
    return [
        LogInfo(msg=(
            "──────── litearm MoveIt demo (planning only) ────────\n"
            "  No control stack: Plan works in RViz, Execute needs an arm.\n"
            "  Real robot: ros2 launch litearm_moveit_config litearm_moveit.launch.py\n"
            "─────────────────────────────────────────────────────")),
        moveit_launch,
    ]


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
