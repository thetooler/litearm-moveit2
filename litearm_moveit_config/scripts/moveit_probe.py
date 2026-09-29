#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""moveit_probe.py — MoveIt 规划链路验收探针。

在已启动的 move_group 上验证：

  1. move_group 起来了，且认识 litearm_arm 这个规划组
  2. 能规划到指定关节目标（OMPL + KDL + SRDF 碰撞矩阵全部生效）
  3. 规划出的轨迹关节名/点数/终点合理
  4. 可选：把规划结果交给 joint_trajectory_controller 真正执行

为什么默认走 /plan_kinematic_path 服务而不是 MoveGroup action
------------------------------------------------------------
MoveGroup action 在 planning_options.plan_only=true 时不会填充 result 里的
planned_trajectory（Humble 版 MoveIt2 的行为），拿不到规划结果做校验。
/plan_kinematic_path（moveit_msgs/srv/GetMotionPlan）则是纯粹的"规划并返回"，
语义明确、不涉及执行，最适合做验收。

真执行只需加 --execute，那时才走 /move_action。

用法：
  ros2 launch litearm_moveit_config litearm_moveit.launch.py dry_run:=true &
  python3 moveit_probe.py                # 只规划（安全）
  python3 moveit_probe.py --execute      # 规划并执行（会驱动机械臂）
"""

import argparse
import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             RobotState, WorkspaceParameters)
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f"joint{i}" for i in range(1, 8)]
GROUP = "litearm_arm"
# 目标位形与 SRDF 的 ready 状态一致。不用全零位是因为它在奇异附近，
# KDL 求逆与 OMPL 采样在那一带都更吃力，失败率偏高会掩盖真正的问题。
GOAL = [0.0, -0.6, 1.1, -0.9, 0.0, 0.7, 0.0]
TOLERANCE_RAD = 0.05


def _build_request() -> MotionPlanRequest:
    request = MotionPlanRequest()
    request.group_name = GROUP
    # 显式指定流水线与规划器：不依赖 default_planning_pipeline 的解析结果，
    # 也就绕开了「只配 planning_plugins 复数时退化成 CHOMP」那个坑
    # （CHOMP 未配置时的表现是"成功但返回空轨迹"，很难定位）。
    request.pipeline_id = "ompl"
    request.planner_id = "RRTConnectkConfigDefault"
    request.num_planning_attempts = 5
    request.allowed_planning_time = 5.0
    request.max_velocity_scaling_factor = 0.2
    request.max_acceleration_scaling_factor = 0.2

    request.workspace_parameters = WorkspaceParameters()
    request.workspace_parameters.header.frame_id = "base_link"
    request.workspace_parameters.min_corner.x = -1.0
    request.workspace_parameters.min_corner.y = -1.0
    request.workspace_parameters.min_corner.z = -1.0
    request.workspace_parameters.max_corner.x = 1.0
    request.workspace_parameters.max_corner.y = 1.0
    request.workspace_parameters.max_corner.z = 1.0

    # 起始状态 is_diff=True + 空内容 → move_group 用当前规划场景的状态
    request.start_state = RobotState()
    request.start_state.is_diff = True

    constraints = Constraints()
    for name, value in zip(JOINTS, GOAL):
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = float(value)
        joint.tolerance_above = TOLERANCE_RAD
        joint.tolerance_below = TOLERANCE_RAD
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    request.goal_constraints.append(constraints)
    return request


def _check_trajectory(node, joint_trajectory) -> int:
    if len(joint_trajectory.points) == 0:
        print("✗ 规划返回了空轨迹")
        return 1
    names = list(joint_trajectory.joint_names)
    print(f"  轨迹关节: {names}")
    print(f"  轨迹点数: {len(joint_trajectory.points)}")
    if sorted(names) != sorted(JOINTS):
        print(f"✗ 轨迹关节集与预期不符（期望 {JOINTS}）")
        return 1
    last = list(joint_trajectory.points[-1].positions)
    order = [names.index(j) for j in JOINTS]
    last = [last[i] for i in order]
    worst = max(abs(last[i] - GOAL[i]) for i in range(7))
    print(f"  终点:     {[round(v, 4) for v in last]}")
    print(f"  目标:     {[round(v, 4) for v in GOAL]}")
    print(f"  终点误差: {worst:.4e} rad（容差 {TOLERANCE_RAD}）")
    duration = joint_trajectory.points[-1].time_from_start
    print(f"  轨迹时长: {duration.sec + duration.nanosec * 1e-9:.3f}s")
    if worst > TOLERANCE_RAD:
        print("✗ 规划终点未落在目标容差内")
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true",
                        help="规划后真正执行（会驱动机械臂，请先确认空间开阔）")
    args = parser.parse_args()

    rclpy.init()
    node = Node("litearm_moveit_probe")
    request = _build_request()

    if not args.execute:
        # ── 仅规划：走 GetMotionPlan 服务 ──
        client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        print("等待 /plan_kinematic_path …")
        if not client.wait_for_service(timeout_sec=40.0):
            print("✗ /plan_kinematic_path 不可用（move_group 未就绪？）")
            return 1
        print("✓ move_group 就绪")

        future = client.call_async(GetMotionPlan.Request(motion_plan_request=request))
        rclpy.spin_until_future_complete(node, future, timeout_sec=60)
        response = future.result()
        if response is None:
            print("✗ 规划服务未在超时内返回")
            return 1
        code = response.motion_plan_response.error_code.val
        print(f"✓ 规划完成: error_code={code} "
              f"({'SUCCESS' if code == 1 else 'FAILURE'}), "
              f"耗时 {response.motion_plan_response.planning_time:.3f}s")
        if code != 1:
            print("✗ 规划失败")
            return 1
        rc = _check_trajectory(
            node, response.motion_plan_response.trajectory.joint_trajectory)
        node.destroy_node()
        rclpy.shutdown()
        if rc != 0:
            return rc
        print("\n✓ MoveIt 规划链路验收通过（未执行，机械臂未运动）")
        return 0

    # ── 规划并执行：走 MoveGroup action ──
    #
    # 验收判据是【/joint_states 是否真的收敛到目标】，而不是 MoveGroup 的
    # 返回码。原因：move_group 的 MoveAction 在执行期间会派生子 goal，rclpy 的
    # ActionClient 偶发会报 "Ignoring unexpected goal response"（收到非当前
    # 序号的目标响应），此时拿到的 result 可能不反映真实执行结果。关节角是
    # 物理事实，比返回码可信。
    live: dict = {}

    def on_joint_state(msg):
        for name, position in zip(msg.name, msg.position):
            live[name] = position

    node.create_subscription(JointState, "/joint_states", on_joint_state, 10)
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(live) >= 7:
        print(f"  起始位置: {[round(live.get(j, float('nan')), 4) for j in JOINTS]}")
    else:
        print(f"✗ /joint_states 未给出 7 个关节（收到 {sorted(live)}）")
        return 1

    client = ActionClient(node, MoveGroup, "/move_action")
    print("等待 /move_action …")
    if not client.wait_for_server(timeout_sec=40.0):
        print("✗ /move_action 不可用（move_group 未就绪？）")
        return 1
    print("✓ move_group 就绪")

    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options.plan_only = False
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.replan = False

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    handle = future.result()
    if handle is None or not handle.accepted:
        print("✗ 规划请求被拒绝")
        return 1
    print("✓ 请求已接受，规划并执行中 …")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=120)
    wrapped = result_future.result()
    if wrapped is not None:
        code = wrapped.result.error_code.val
        print(f"  MoveGroup 返回码: {code} "
              f"({'SUCCESS' if code == 1 else 'FAILURE/其他'})")
    else:
        print("  MoveGroup 未在超时内返回结果（继续按关节角判定）")

    # 轨迹时长 3s 量级，再多给几秒让跟踪收敛
    for _ in range(120):
        rclpy.spin_once(node, timeout_sec=0.05)

    worst = 0.0
    for index, name in enumerate(JOINTS):
        worst = max(worst, abs(live.get(name, 99.0) - GOAL[index]))
    print(f"  实测位置: {[round(live.get(j, float('nan')), 4) for j in JOINTS]}")
    print(f"  目标位置: {[round(v, 4) for v in GOAL]}")
    print(f"  最大误差: {worst:.4e} rad（容差 {TOLERANCE_RAD}）")

    node.destroy_node()
    rclpy.shutdown()
    if worst >= TOLERANCE_RAD:
        print("✗ 执行后关节角未收敛到目标")
        return 1
    print("\n✓ MoveIt 规划 + 执行链路验收通过（关节角已收敛到目标）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
