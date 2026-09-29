#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_moveit.launch.py — 起 litearm 的 MoveIt 规划栈（可选含底层控制栈）。

按需组合：

    1. 控制栈   litearm_ros2_control/litearm_control.launch.py
                （硬件守护进程 + controller_manager + JSB + JTC）
    2. move_group   MoveIt 规划节点（OMPL + KDL）
    3. RViz     MoveIt 运动规划面板

默认三件一起起。若控制栈已在别处运行（例如真机调试时手动起的），
用 start_control:=false 只起 MoveIt 部分，避免两个 controller_manager
抢同一批命令接口。

    # 真机（板子已上电、USB 已连、license 已激活）
    ros2 launch litearm_moveit_config litearm_moveit.launch.py

    # 无硬件全链路演练
    ros2 launch litearm_moveit_config litearm_moveit.launch.py dry_run:=true

    # 只起 MoveIt（控制栈已在运行）
    ros2 launch litearm_moveit_config litearm_moveit.launch.py start_control:=false

前馈由**固件**算（无需在这里开任何开关）：PD 增益与重力/摩擦/积分/kd_extra
前馈都在 litearm-stm32 固件里，模型由 URDF 生成、编译进固件。默认通道
（MOVE_JS）下**没有 M·q̈ 与 C·q̇**——MOVE_JS 没有加速度源。

下面五个开关是**三态**，默认空 = 不碰固件（固件出厂掩码已带 G/惯量/科氏/摩擦/
积分/量化/速度参考）：

    gravity_compensation:=true|false    覆盖 FF_G
    friction_compensation:=true|false   覆盖 FF_FRICTION
    inertia_compensation:=true|false    覆盖 FF_INERTIA|FF_CORIOLIS（MOVE_JS 下无效）
    integral_compensation:=true|false   覆盖 FF_INTEGRAL
    damping_compensation:=true|false    覆盖 kd_extra 向量（true = 恢复出厂值）

要退回纯 PD（例如 A/B 对照），五项都给 false：
    ros2 launch litearm_moveit_config litearm_moveit.launch.py \\
        gravity_compensation:=false friction_compensation:=false \\
        inertia_compensation:=false integral_compensation:=false \\
        damping_compensation:=false

想自己算前馈（KP/Kd/effort 逐帧生效）请用控制栈的 mit_passthrough:=true，
并把 joint_trajectory_controller 的 command_interfaces 改成相应组合。
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

from litearm_ros2_control.ros_env import isolation_hint, parse_bool, ros_isolation_env


def _is_true(value: str) -> bool:
    return parse_bool(value, False)


def _declare_arguments():
    return [
        DeclareLaunchArgument("start_control", default_value="true",
                              description="是否一并启动 ros2_control 控制栈"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="透传给控制栈：true = 无硬件模式"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="是否启动 RViz2（MoveIt 运动规划面板）"),
        DeclareLaunchArgument("port", default_value="",
                              description="透传给控制栈：litearm-stm32 的 USB CDC "
                                          "设备路径，留空 = 自动发现（1d50:606f）"),
        DeclareLaunchArgument("shm_name", default_value="/litearm_hw",
                              description="透传给控制栈：共享内存段名"),
        DeclareLaunchArgument("daemon_rate_hz", default_value="0",
                              description="透传给控制栈：守护进程命令下发频率，0 = 用默认值"),
        # 五个前馈开关是**三态**：默认空 = 不碰固件（固件自己有出厂掩码，
        # 出厂就带 G/惯量/科氏/摩擦/积分/量化/速度参考）。
        # 换底层之前这里是 true（守护进程在 tau_ff 上自己叠前馈）；现在前馈
        # 由固件算，再传 true 只是"把已经开着的位再置一次"——无害但没意义，
        # 反而会让人以为 MoveIt 入口和直接起控制栈是两套行为。
        DeclareLaunchArgument("gravity_compensation", default_value="",
                              description="透传给控制栈：覆盖固件 ff_mask 的 FF_G 位"
                                          "（空 = 不碰固件，true/false = 置位/清位）"),
        DeclareLaunchArgument("friction_compensation", default_value="",
                              description="透传给控制栈：覆盖固件 ff_mask 的 FF_FRICTION 位"),
        DeclareLaunchArgument("inertia_compensation", default_value="",
                              description="透传给控制栈：覆盖固件 ff_mask 的 "
                                          "FF_INERTIA|FF_CORIOLIS 位（⚠ MOVE_JS 通道下"
                                          "固件不算惯量项，置位无用）"),
        DeclareLaunchArgument("integral_compensation", default_value="",
                              description="透传给控制栈：覆盖固件 ff_mask 的 FF_INTEGRAL 位"),
        DeclareLaunchArgument("damping_compensation", default_value="",
                              description="透传给控制栈：覆盖固件 kd_extra 向量"
                                          "（false = 清零，true = 恢复出厂值）"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="move_group 日志级别"),
        DeclareLaunchArgument("ros_domain_id", default_value="42",
                              description="本栈使用的 ROS 域（默认刻意避开 0）。"
                                          "要与外部系统对接时设 0"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = 只在本机发现。跨机串扰会让 RViz "
                                          "显示别人的机器人、/move_action 出现多个 server"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    start_control = _is_true(resolve("start_control"))
    use_rviz = _is_true(resolve("use_rviz"))
    shm_name = resolve("shm_name")
    dry_run = resolve("dry_run")
    port = resolve("port")
    daemon_rate_hz = resolve("daemon_rate_hz")
    gravity_compensation = resolve("gravity_compensation")
    friction_compensation = resolve("friction_compensation")
    inertia_compensation = resolve("inertia_compensation")
    integral_compensation = resolve("integral_compensation")
    damping_compensation = resolve("damping_compensation")
    domain_id = resolve("ros_domain_id").strip() or "42"
    localhost_only = resolve("ros_localhost_only")
    env = ros_isolation_env(domain_id, localhost_only)

    moveit_share = get_package_share_directory("litearm_moveit_config")
    control_share = get_package_share_directory("litearm_ros2_control")

    # shm_name 必须与控制栈一致：MoveIt 侧展开 URDF 时同样要注入，
    # 否则 move_group 拿到的 robot_description 与真正加载的硬件插件参数不符。
    moveit_config = (
        MoveItConfigsBuilder("litearm", package_name="litearm_moveit_config")
        .robot_description(mappings={"litearm_shm_name": shm_name})
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    actions = []

    if start_control:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_share, "launch", "litearm_control.launch.py")),
            launch_arguments={
                "dry_run": dry_run,
                "port": port,
                "shm_name": shm_name,
                "daemon_rate_hz": daemon_rate_hz,
                "gravity_compensation": gravity_compensation,
                "friction_compensation": friction_compensation,
                "inertia_compensation": inertia_compensation,
                "integral_compensation": integral_compensation,
                "damping_compensation": damping_compensation,
                "use_rviz": "false",  # 控制栈自带的 RViz 与下面的 MoveIt RViz 二选一
                # 隔离参数必须转发：被 include 的进程要落在同一个域里，
                # 否则 move_group 根本看不到 controller_manager 的 action。
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
        # 是否起 RViz 只用上面这个 Python 判断，**不要**再加运行时
        # IfCondition(LaunchConfiguration("use_rviz"))：ROS 2 launch 把
        # IncludeLaunchDescription 的 launch_arguments 实现为
        # SetLaunchConfiguration，写入的是共享上下文 —— 上面控制栈的 include
        # 显式传了 use_rviz:=false（"控制栈 RViz 与 MoveIt RViz 二选一"），
        # 会把本函数的 use_rviz 配置覆盖成 false，运行时条件随之恒假，
        # RViz 被静默跳过（本工程踩过：launch 日志里连 rviz2 进程都没有）。
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(moveit_share, "rviz", "moveit.rviz")],
            output="log",
            additional_env=env,
            # RViz 的 MoveIt 插件要自己拿到这几组参数，不能只依赖 move_group。
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ))

    banner_target = ("dry-run（pty 假固件）" if _is_true(dry_run)
                     else f"真机，端口={port or '(自动发现)'}")
    actions.insert(0, LogInfo(msg=(
        "──────── litearm MoveIt ────────\n"
        f"  {banner_target}\n"
        "  ⚠ 本 launch 已锁 ROS 域；想在终端里用 ros2 命令行 / 看 RViz 数据，\n"
        "    请先在你自己的终端里执行：\n"
        f"        {isolation_hint(domain_id, localhost_only)}\n"
        "────────────────────────────────")))
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
