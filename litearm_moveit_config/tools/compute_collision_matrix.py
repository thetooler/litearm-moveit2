#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""compute_collision_matrix.py — 用采样法生成 SRDF 的 disable_collisions 列表。

为什么需要它
------------
MoveIt 的 SRDF 必须告诉它"哪些 link 对不需要做自碰撞检查"。写错的两个方向
代价不对称：

* **漏禁用**（把一对永久贴合的 link 留着检查）→ MoveIt 认为起始状态就自碰撞，
  所有规划直接失败，配置完全不可用。
* **多禁用**（把一对会接近但不会碰的 link 关掉）→ 少一层保护。

因此本脚本在阈值上偏保守（默认 20mm），并把每个被禁用的 link 对连同
"采样到的最小距离"一起输出，方便人工复核。

算法
----
就是 moveit_setup_assistant 的做法，只是用 numpy/scipy 自己实现：

1. 解析 URDF：link 链、joint origin、collision 网格与 origin
2. 读二进制 STL，去重顶点并按步长抽稀（默认每件 ≤ 12000 点）
3. 在零位 + 若干随机合法位形下做正运动学
4. 对每个 link 对，把 A 的顶点变换到世界系，用 cKDTree 查它到 B 各顶点的最近距离
5. 若任意采样位形下最小距离 < 阈值，则禁用该对

用法::

    # 默认：用 ament_index 定位已安装的 litearm 包（不依赖任何绝对路径）
    python3 compute_collision_matrix.py --output srdf_snippet.xml

    # 或显式指定
    python3 compute_collision_matrix.py \\
        --urdf <litearm.urdf> --mesh-root <包含 meshes/ 的包目录> \\
        --output srdf_snippet.xml

输出为可直接粘进 SRDF 的 <disable_collisions> 片段。
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial import cKDTree


def _default_share_dir() -> str:
    """通过 ament_index 定位 litearm 描述包（等价于 xacro 里的 $(find litearm)）。"""
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:  # pragma: no cover - 环境问题而非逻辑分支
        raise SystemExit(f"需要 ament_index_python，请先 source ROS 环境：{exc}")
    try:
        return get_package_share_directory("litearm")
    except Exception as exc:  # pragma: no cover
        raise SystemExit(
            f"ament_index 找不到 litearm 包（colcon build 并 source install/setup.bash 了吗？）：{exc}")

# URDF 的 rpy 约定：固定轴 XYZ 外旋，等价于 R = Rz(y)Ry(p)Rx(r)
def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12 or abs(angle) < 1e-15:
        return np.eye(3)
    k = axis / norm
    kx, ky, kz = k
    c, s = math.cos(angle), math.sin(angle)
    v = 1.0 - c
    return np.array([
        [c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v],
    ])


def read_binary_stl(path: str, max_points: int) -> np.ndarray:
    """读二进制 STL，返回去重并抽稀后的顶点 (N,3) float64。"""
    with open(path, "rb") as handle:
        header = handle.read(80)
        if header[:5].lower() == b"solid" and not header[80:84]:
            pass  # 头部以 solid 开头也可能是二进制，按三角面计数判断
        raw_count = handle.read(4)
        if len(raw_count) < 4:
            raise ValueError(f"{path}: 文件过短，不是合法 STL")
        triangles = int(np.frombuffer(raw_count, dtype="<u4")[0])
        payload = handle.read(triangles * 50)
    if len(payload) < triangles * 50:
        raise ValueError(f"{path}: 二进制 STL 数据不完整")
    record = np.frombuffer(payload, dtype=np.uint8).reshape(triangles, 50)
    # 每条记录：法线(3×f4) + 3 顶点(9×f4) + 属性(2B)
    floats = record[:, :48].copy().view("<f4").reshape(triangles, 12)
    vertices = floats[:, 3:12].reshape(-1, 3).astype(np.float64)

    vertices = np.unique(vertices, axis=0)
    if max_points > 0 and len(vertices) > max_points:
        step = int(math.ceil(len(vertices) / max_points))
        vertices = vertices[::step]
    return vertices


class LinkGeometry:
    """一个 link 的碰撞点云与其在 link 系下的位姿。"""

    def __init__(self, name: str, points: np.ndarray, origin: np.ndarray) -> None:
        self.name = name
        self.points = points          # (N,3) link 坐标系
        self.origin = origin          # 4x4，collision origin 相对 link 系


class UrdfModel:
    """够用的 URDF 解析 + 正运动学（只处理 revolute/continuous/fixed 串链）。"""

    def __init__(self, urdf_path: str, mesh_root: str, max_points: int) -> None:
        root = ET.parse(urdf_path).getroot()
        self.mesh_root = mesh_root.rstrip("/")

        self.links: Dict[str, LinkGeometry] = {}
        for link in root.findall("link"):
            name = link.get("name")
            collision = link.find("collision")
            if collision is None:
                continue
            geometry = collision.find("geometry")
            mesh = geometry.find("mesh") if geometry is not None else None
            if mesh is None:
                # 非 mesh 碰撞体（box/cylinder）本工程没用到，跳过即可
                continue
            filename = mesh.get("filename")
            local = filename.split("package://")[-1]
            local = local.split("/", 1)[1] if "/" in local else local
            path = f"{self.mesh_root}/{local}"
            points = read_binary_stl(path, max_points)
            origin_el = collision.find("origin")
            origin = self._origin_matrix(origin_el)
            self.links[name] = LinkGeometry(name, points, origin)

        self.joints = []      # (name, parent, child, origin4x4, axis, type)
        self.child_joint: Dict[str, Tuple[str, np.ndarray, np.ndarray, str]] = {}
        for joint in root.findall("joint"):
            jtype = joint.get("type")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            origin = self._origin_matrix(joint.find("origin"))
            axis_el = joint.find("axis")
            axis = (np.array([float(v) for v in axis_el.get("xyz").split()])
                    if axis_el is not None else np.array([1.0, 0.0, 0.0]))
            self.joints.append((joint.get("name"), parent, child, origin, axis, jtype))
            if jtype in ("revolute", "continuous"):
                index = self._revolute_index(joint.get("name"))
                self.child_joint[child] = (joint.get("name"), parent, origin, axis, index, jtype)
            else:
                self.child_joint[child] = (joint.get("name"), parent, origin, axis, -1, jtype)

        self.root_link = self._find_root(root)
        # 按链顺序排列（从根开始），保证父子位姿先算出来
        self.order = self._chain_order()

    def _revolute_index(self, joint_name: str) -> int:
        """jointN → 数组下标 N-1。"""
        digits = "".join(ch for ch in joint_name if ch.isdigit())
        return int(digits) - 1

    @staticmethod
    def _origin_matrix(origin_el: Optional[ET.Element]) -> np.ndarray:
        matrix = np.eye(4)
        if origin_el is None:
            return matrix
        xyz = origin_el.get("xyz", "0 0 0")
        rpy = origin_el.get("rpy", "0 0 0")
        matrix[:3, 3] = [float(v) for v in xyz.split()]
        matrix[:3, :3] = rpy_to_matrix(*[float(v) for v in rpy.split()])
        return matrix

    @staticmethod
    def _find_root(root: ET.Element) -> str:
        children = {j.find("child").get("link") for j in root.findall("joint")}
        for link in root.findall("link"):
            if link.get("name") not in children:
                return link.get("name")
        raise ValueError("URDF 无根 link")

    def _chain_order(self) -> List[str]:
        order, stack = [], [self.root_link]
        while stack:
            link = stack.pop(0)
            order.append(link)
            for child, (_jn, parent, *_rest) in self.child_joint.items():
                if parent == link:
                    stack.append(child)
        return order

    def link_poses(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        """给定关节角返回每个 link 相对世界（根 link）的 4x4 位姿。"""
        poses = {self.root_link: np.eye(4)}
        for link in self.order[1:]:
            _name, parent, origin, axis, index, jtype = self.child_joint[link]
            parent_pose = poses[parent]
            joint_pose = origin.copy()
            if jtype in ("revolute", "continuous") and index >= 0:
                angle = float(q[index]) if index < len(q) else 0.0
                rotation = np.eye(4)
                rotation[:3, :3] = axis_angle_to_matrix(axis, angle)
                joint_pose = origin @ rotation
            poses[link] = parent_pose @ joint_pose
        return poses

    def world_points(self, link: str, q: np.ndarray,
                     poses: Optional[Dict[str, np.ndarray]] = None) -> np.ndarray:
        """link 的碰撞点云在世界系下的坐标。"""
        poses = poses if poses is not None else self.link_poses(q)
        geometry = self.links[link]
        transform = poses[link] @ geometry.origin
        pts = geometry.points
        return pts @ transform[:3, :3].T + transform[:3, 3]


def sample_configurations(rng: np.random.Generator, limits: np.ndarray,
                          count: int) -> List[np.ndarray]:
    """零位 + 随机合法位形。零位一定要在内：SRDF 的 Default 判定依赖它。"""
    configs = [np.zeros(limits.shape[0])]
    if count > 1:
        low, high = limits[:, 0], limits[:, 1]
        configs.extend(low + (high - low) * rng.random((count - 1, limits.shape[0])))
    return configs


def joint_limits(urdf_path: str) -> Tuple[List[str], np.ndarray]:
    root = ET.parse(urdf_path).getroot()
    names, limits = [], []
    for joint in root.findall("joint"):
        if joint.get("type") not in ("revolute", "continuous"):
            continue
        name = joint.get("name")
        limit = joint.find("limit")
        if limit is None:
            lower, upper = -math.pi, math.pi
        else:
            lower = float(limit.get("lower", -math.pi))
            upper = float(limit.get("upper", math.pi))
            if joint.get("type") == "continuous":
                lower, upper = -math.pi, math.pi
        names.append(name)
        limits.append((lower, upper))
    # 按 jointN 排序，与正运动学的下标约定一致
    order = sorted(range(len(names)), key=lambda i: names[i])
    return [names[i] for i in order], np.array([limits[i] for i in order])


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--urdf", default="",
                        help="URDF 路径；留空用 ament_index 定位已安装的 litearm 包")
    parser.add_argument("--mesh-root", default="",
                        help="对应 package://<pkg>/ 的本地目录 "
                             "（即包含 meshes/ 的那一级，不是 meshes/ 本身）；"
                             "留空用 ament_index 定位已安装的 litearm 包")
    parser.add_argument("--samples", type=int, default=120,
                        help="随机位形采样数（含零位，默认 120）")
    parser.add_argument("--max-points", type=int, default=12000,
                        help="每个 link 的顶点抽稀上限（默认 12000）")
    parser.add_argument("--touch-epsilon", type=float, default=0.002,
                        help="判定「所有位形下都接触」的最近距离阈值(m)，默认 0.002。"
                             "点云近似下接触对的实测值一般 <1mm")
    parser.add_argument("--default-epsilon", type=float, default=0.005,
                        help="判定「零位接触」的最近距离阈值(m)，默认 0.005。"
                             "这类对不禁用会让 MoveIt 认为起始状态自碰撞")
    parser.add_argument("--seed", type=int, default=20260917, help="采样随机种子")
    parser.add_argument("--output", default="", help="输出文件；留空打印到 stdout")
    args = parser.parse_args(argv)

    if not args.urdf or not args.mesh_root:
        default_share = _default_share_dir()
        args.urdf = args.urdf or os.path.join(default_share, "urdf", "litearm.urdf")
        args.mesh_root = args.mesh_root or default_share

    model = UrdfModel(args.urdf, args.mesh_root, args.max_points)
    names, limits = joint_limits(args.urdf)
    print(f"# URDF: {args.urdf}", file=sys.stderr)
    print(f"# link（含碰撞体）: {sorted(model.links)}", file=sys.stderr)
    print(f"# 关节: {names}", file=sys.stderr)
    print(f"# 采样 {args.samples} 个位形（含零位）；接触阈值 "
          f"Always<{args.touch_epsilon * 1000:.0f}mm / "
          f"Default<{args.default_epsilon * 1000:.0f}mm；"
          f"每 link ≤{args.max_points} 点", file=sys.stderr)

    rng = np.random.default_rng(args.seed)
    configs = sample_configurations(rng, limits, args.samples)

    # 预计算每个采样位形下的世界点云
    cache: List[Dict[str, np.ndarray]] = []
    for index, q in enumerate(configs):
        poses = model.link_poses(q)
        cache.append({link: model.world_points(link, q, poses)
                      for link in model.links})
        if (index + 1) % 20 == 0:
            print(f"#   位形 {index + 1}/{len(configs)}", file=sys.stderr)

    links = [l for l in model.order if l in model.links]

    # 按采样位形预建 KD-tree 并缓存：每个 link 的树会被多个 link 对复用，
    # 若在配对循环里重建，代价是 O(对数 × 采样数) 次建树，实测慢 3 倍以上。
    trees: List[Dict[str, Tuple[np.ndarray, cKDTree]]] = [
        {link: (cloud[link], cKDTree(cloud[link])) for link in links}
        for cloud in cache
    ]

    # 结构相邻对（由同一个 joint 直接相连）：碰撞网格在关节处天然互相穿插，
    # 这是建模方式决定的，与位形无关，一律禁用。
    adjacent = set()
    for _name, parent, child, _origin, _axis, _jtype in model.joints:
        if parent in model.links and child in model.links:
            adjacent.add(tuple(sorted((parent, child))))

    results = []  # (a, b, min_over_samples, min_at_zero, contact_samples, is_adjacent)
    total_samples = len(configs)
    for i, a in enumerate(links):
        for b in links[i + 1:]:
            best_all = math.inf
            best_zero = math.inf
            contact = 0
            for index, entry in enumerate(trees):
                pts_a, _ = entry[a]
                _pts_b, tree_b = entry[b]
                distance, _ = tree_b.query(pts_a, k=1, workers=-1)
                minimum = float(distance.min())
                if index == 0:
                    best_zero = minimum
                if minimum < args.touch_epsilon:
                    contact += 1
                if minimum < best_all:
                    best_all = minimum
            results.append((a, b, best_all, best_zero, contact,
                            tuple(sorted((a, b))) in adjacent))

    # 分三类，与 moveit_setup_assistant 的分类语义一致：
    #
    #   Adjacent —— 结构相邻，必然穿插，禁用
    #   Always   —— 【每个】采样位形下都接触/穿插，禁用
    #   Default  —— 零位就接触/穿插（例如某两段壳体在零位贴合），禁用，
    #               否则 MoveIt 会认为起始状态即自碰撞，规划全部失败
    #
    # 关键：判据是「接触样本数」而不是「采样最小距离」。
    # 最小距离小只说明【存在】某些位形下两段壳体贴到一起，那恰恰是 MoveIt
    # 需要检查并避开的区域——按最小距离去禁用会把这个保护关掉。
    adjacent_pairs, always_pairs, default_pairs, kept = [], [], [], []
    for a, b, best_all, best_zero, contact, is_adjacent in results:
        if is_adjacent:
            adjacent_pairs.append((a, b))
        elif contact == total_samples:
            always_pairs.append((a, b, best_all))
        elif best_zero < args.default_epsilon:
            default_pairs.append((a, b, best_zero))
        else:
            kept.append((a, b, best_all, best_zero, contact, False))

    lines = [
        "<!-- 由 tools/compute_collision_matrix.py 生成，请勿手工维护。",
        f"     方法：{len(configs)} 个位形（零位 + 随机）下的点云最近距离；",
        f"     每 link 至多 {args.max_points} 个抽稀顶点。",
        f"     判定：相邻=结构穿插；Always=每个位形都 <{args.touch_epsilon * 1000:.0f}mm；",
        f"          Default=零位 <{args.default_epsilon * 1000:.0f}mm。",
        "     仅「某些位形下会接近/接触」的对【不】禁用——那正是需要 MoveIt",
        "     检查并避开的区域；按最小距离禁用会关掉这层保护。",
        "     这是点云近似，非精确网格距离；改 URDF 后需重新生成。 -->",
    ]
    for a, b in sorted(adjacent_pairs):
        lines.append(f'    <disable_collisions link1="{a}" link2="{b}" '
                     f'reason="Adjacent"/>')
    for a, b, distance in sorted(always_pairs, key=lambda item: item[2]):
        lines.append(f'    <disable_collisions link1="{a}" link2="{b}" '
                     f'reason="Always (MinDistance {distance * 1000:.1f}mm)"/>')
    for a, b, distance in sorted(default_pairs, key=lambda item: item[2]):
        lines.append(f'    <disable_collisions link1="{a}" link2="{b}" '
                     f'reason="Default (zero-pose MinDistance '
                     f'{distance * 1000:.1f}mm)"/>')

    text = "\n".join(lines) + "\n"
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(text)
        print(f"# 已写入 {args.output}", file=sys.stderr)
    else:
        print(text)

    total_disabled = len(adjacent_pairs) + len(always_pairs) + len(default_pairs)
    print(f"# 共 {len(results)} 对候选：相邻 {len(adjacent_pairs)}、"
          f"全位形接触 {len(always_pairs)}、零位接触 {len(default_pairs)}，"
          f"禁用合计 {total_disabled}，其余 {len(kept)} 对保留检查", file=sys.stderr)
    if kept:
        print(f"# 保留检查的对（共 {len(kept)} 对；按「曾接触过的位形数」降序，"
              f"前 10 对）：", file=sys.stderr)
        for a, b, best_all, _best_zero, contact, _adj in sorted(
                kept, key=lambda i: (-i[4], i[2]))[:10]:
            print(f"#   {a:10s} {b:10s} 接触位形 {contact:3d}/{total_samples} "
                  f"最小距离 {best_all * 1000:7.1f}mm", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
