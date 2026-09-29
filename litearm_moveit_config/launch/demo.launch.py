#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""demo.launch.py — 不接真实硬件的 MoveIt 演示。

一条命令起 MoveIt 面板，在 RViz 里用鼠标拖目标位形、点 Plan & Execute
看机械臂真的"动"起来：

    ros2 launch litearm_moveit_config demo.launch.py

本质上就是把 litearm_moveit.launch.py 的 dry_run 硬钉为 true。
单独做一个入口的理由和 litearm_demo.launch.py 一样：**把"演示"和"真机"分成
两个命令**，就不存在看错参数把真臂开起来这种事。

真机请用：
    ros2 launch litearm_moveit_config litearm_moveit.launch.py
（板子已上电、USB 已连、license 已激活，并确认机械臂已支撑、急停可触达）

RViz 里可用的命名状态（SRDF 的 group_state）：

    zero    全零位
    ready   肘部抬起的舒展构型，比零位更适合作为规划起点
            （零位在奇异附近，笛卡尔规划容易失败）

无显示环境请加 use_rviz:=false，改用命令行验证：
    ros2_ws/src/litearm_moveit_config/scripts/acceptance_moveit.sh --execute
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
                              description="是否启动 RViz2（MoveIt 运动规划面板）"),
        DeclareLaunchArgument("shm_name", default_value="/litearm_hw",
                              description="共享内存段名（演示不必改）"),
        DeclareLaunchArgument("ros_domain_id", default_value="42",
                              description="本演示使用的 ROS 域（默认刻意避开 0）"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = 只在本机发现，避免跨机串扰"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("litearm_moveit_config") + "/launch/litearm_moveit.launch.py"),
        launch_arguments={
            # 硬钉：这个入口永远是模拟（pty 假固件），不会碰真硬件
            "dry_run": "true",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "shm_name": resolve("shm_name"),
            "ros_domain_id": resolve("ros_domain_id"),
            "ros_localhost_only": resolve("ros_localhost_only"),
        }.items())
    return [
        LogInfo(msg=(
            "──────── litearm MoveIt 无硬件演示 ────────\n"
            "  模式：dry-run（守护进程在 pty 上起假固件，链路走真实协议）\n"
            "  RViz 里用 MotionPlanning 面板拖目标 → Plan & Execute\n"
            "  真机请用：ros2 launch litearm_moveit_config litearm_moveit.launch.py\n"
            "──────────────────────────────────────────")),
        moveit_launch,
    ]


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
