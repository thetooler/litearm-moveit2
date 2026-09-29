#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_from_firmware_config.py — 从 litearm-stm32 固件参数表派生 MoveIt 配置。

为什么要派生而不是手抄
----------------------
``config/joint_limits.yaml`` 里的限位/速度/加速度全部来自**固件参数表**
（``User/litearm/params/defaults.c`` 与它 include 的 ``joint_limit_macros.h``）：

* 位置限位 ← ``JL_QMIN_Jn / JL_QMAX_Jn``（由 ``tools/export_model.py`` 从 URDF
  硬限位内缩 1° 生成）。注意这比 URDF 里的 ``<limit>`` **刻意收窄 1°**
  （URDF 是 -2.827，这里是 -2.809547），那是工程自己的安全余量。MoveIt 的规划
  空间应该与**固件运行期的安全包络**一致，否则会规划出界、被固件拒绝，表现为
  "规划成功但执行失败"。
* 速度/加速度/加加速度 ← ``joint[].vel_max / acc_max / jerk``

jerk 上限不只是"多一个约束"：``AddRuckigTrajectorySmoothing``（双 S 曲线、
jerk 受限的速度规划）在没有 jerk 限位时会退回内置默认值，而不是工程调过的
那组包络——于是 MoveIt 与固件的 move_j S 曲线在加加速度上根本不是同一条约束。
必须一并搬过来。

手抄这些 7×4 个数极易出错，而且改了固件参数后不会有任何提示。
所以做成可重复执行的生成器：**改了固件参数表就重跑一次**。

为什么期望源是固件而不是 pylitearm
----------------------------------
换底层之后关节限位与运动包络的权威来源是固件参数表（守护进程启动时用 0x24
把这些值读回去，见 hw_daemon.py 的「固件身份」日志）。ROS 侧不该自己另有一套
答案——那正是"改了没反应、且很难查"的来源。

用法::

    python3 gen_from_firmware_config.py                    # 自动找 litearm-stm32
    python3 gen_from_firmware_config.py --firmware-root <path>
    python3 gen_from_firmware_config.py --check            # 只校验同步，不写文件

找不到固件仓库时：``--check`` 返回 2（无法校验，与"不同步"区分开），
不带 ``--check`` 时直接报错退出——**不写一份凭猜测的值**。
"""

import argparse
import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence

NUM_JOINTS = 7

HEADER = """# joint_limits.yaml — MoveIt 规划用关节限位。
#
# 数值全部来自 litearm-stm32 固件的参数表
# （User/litearm/params/defaults.c 与 joint_limit_macros.h），不是 URDF 里的
# 原始值：固件的限位刻意比 URDF 收窄 1°，那是工程自己的安全余量。
# 让 MoveIt 的规划空间与固件运行期的安全包络一致，避免规划出界后被固件拒绝。
#
# jerk 限位供 AddRuckigTrajectorySmoothing（双 S 曲线）使用；缺了它 Ruckig
# 会用自己的默认值，与固件 move_j 的 S 曲线包络不一致。
#
# ⚠️ 本文件由 tools/gen_from_firmware_config.py 生成，请勿手工维护。
#    改固件参数表后重跑：
#      python3 tools/gen_from_firmware_config.py --firmware-root <litearm-stm32>
"""

INITIAL_HEADER = """# initial_positions.yaml — 启动/RViz 交互的初始关节角。
#
# 全零位（与固件的零点一致）。
# 由 tools/gen_from_firmware_config.py 生成。
"""


def find_firmware_root(explicit: Optional[str] = None) -> Optional[str]:
    """定位 litearm-stm32 仓库；找不到返回 ``None``。

    它是**另一个仓库**，不在本工作区里，所以按约定位置找 + 允许显式覆盖。
    """
    candidates = [explicit, os.environ.get("LITEARM_STM32_ROOT"),
                  os.path.join(os.path.expanduser("~"), "wkspace",
                               "litearm-stm32")]
    # <ws>/src/<pkg>/tools → 上溯到 <ws>，再找同级的 litearm-stm32
    here = Path(__file__).resolve()
    workspace = here.parents[3]          # src/<pkg>/tools/xxx.py → <ws>
    candidates.append(str(workspace.parent / "litearm-stm32"))
    for path in candidates:
        if path and os.path.isdir(path):
            return path
    return None


def _macros(path: Path) -> Dict[str, float]:
    """读 ``#define JL_XXX_Jn (v f)`` 形式的限位宏。"""
    out: Dict[str, float] = {}
    text = path.read_text(encoding="utf-8")
    for name, value in re.findall(
            r"#define\s+(JL_\w+)\s+\(\s*(-?[\d.]+)f\s*\)", text):
        out[name] = float(value)
    return out


def _joint_table(text: str) -> Dict[int, Dict[str, object]]:
    """从 ``defaults.c`` 的文本里抽出 7 关节参数表。

    按 ``.can_id=0x`` 切块，再用 ``q_min=JL_QMIN_Jn`` 认关节号——**不能用
    can_id 认**：单关节台架表（``#if LITEARM_BENCH_1J``）的 can_id 也是 0x01，
    会把台架那组参数混进来；而台架表的 q_min 是字面量（-12.0f），拿它当判据
    天然排除。
    """
    joints: Dict[int, Dict[str, object]] = {}
    for block in text.split(".can_id=0x")[1:]:
        match = re.search(r"q_min=(JL_QMIN_J\d)", block)
        if not match:
            continue
        q_min_macro = match.group(1)
        q_max_match = re.search(r"q_max=(JL_QMAX_J\d)", block)
        if not q_max_match:
            raise ValueError(f"关节块里有 {q_min_macro} 却没有对应的 q_max 宏")
        joints[int(q_min_macro[-1]) - 1] = {
            "q_min_macro": q_min_macro,
            "q_max_macro": q_max_match.group(1),
            "vel_max": _field(block, "vel_max"),
            "acc_max": _field(block, "acc_max"),
            "jerk": _field(block, "jerk"),
        }
    return joints


def _field(block: str, name: str) -> float:
    """取 ``.name=1.0f`` 的数值。前导 ``\\.`` 让 ``.tau_max`` 不误匹配 ``.wall_tau_max``。"""
    match = re.search(rf"\.{name}=(-?[\d.]+(?:[eE][-+]?\d+)?)f", block)
    if not match:
        raise ValueError(f"关节块里找不到 .{name}=（源码结构变了？）")
    return float(match.group(1))


def load_joints(root: str) -> List[Dict[str, float]]:
    """读出 7 个关节的限位与运动包络；结构不符即抛 ValueError。"""
    root_path = Path(root)
    macros = _macros(root_path / "User" / "litearm" / "params"
                     / "joint_limit_macros.h")
    text = (root_path / "User" / "litearm" / "params" / "defaults.c"
            ).read_text(encoding="utf-8")
    table = _joint_table(text)
    if sorted(table) != list(range(NUM_JOINTS)):
        raise ValueError(
            f"从 defaults.c 解析出 {sorted(table)} 号关节，期望 0..{NUM_JOINTS - 1}"
            f"（解析器要跟上源码结构）")

    out: List[Dict[str, float]] = []
    for index in range(NUM_JOINTS):
        entry = table[index]
        for macro_key in ("q_min_macro", "q_max_macro"):
            name = entry[macro_key]
            if name not in macros:
                raise ValueError(f"joint_limit_macros.h 里找不到 {name}")
        out.append({
            "q_min": macros[entry["q_min_macro"]],
            "q_max": macros[entry["q_max_macro"]],
            "vel_max": entry["vel_max"],
            "acc_max": entry["acc_max"],
            "jerk": entry["jerk"],
        })
    return out


def render_joint_limits(joints: List[Dict[str, float]]) -> str:
    lines: List[str] = [HEADER]
    lines.append("joint_limits:")
    for index, joint in enumerate(joints):
        lines += [
            f"  joint{index + 1}:",
            "    has_position_limits: true",
            f"    min_position: {joint['q_min']:.6f}",
            f"    max_position: {joint['q_max']:.6f}",
            "    has_velocity_limits: true",
            f"    max_velocity: {joint['vel_max']:.6f}",
            "    has_acceleration_limits: true",
            f"    max_acceleration: {joint['acc_max']:.6f}",
            "    has_jerk_limits: true",
            f"    max_jerk: {joint['jerk']:.6f}",
            "",
        ]
    return "\n".join(lines)


def render_initial_positions() -> str:
    return INITIAL_HEADER + "initial_positions:\n" + "".join(
        f"  joint{i}: 0.0\n" for i in range(1, NUM_JOINTS + 1))


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--firmware-root", default=None,
                        help="litearm-stm32 仓库根（默认自动找；"
                             "也可用环境变量 LITEARM_STM32_ROOT）")
    parser.add_argument("--output-dir",
                        default=str(Path(__file__).resolve().parents[1] / "config"),
                        help="输出目录（默认本包的 config/）")
    parser.add_argument("--check", action="store_true",
                        help="只校验现有文件是否与派生结果一致，不写入")
    args = parser.parse_args(list(argv) if argv is not None else None)

    root = find_firmware_root(args.firmware_root)
    if root is None:
        if args.check:
            print("⚠ 找不到 litearm-stm32 仓库，无法校验（返回 2 与「不同步」区分）")
            return 2
        print("✗ 找不到 litearm-stm32 仓库。用 --firmware-root 指定，或设 "
              "LITEARM_STM32_ROOT。\n"
              "  **不会凭猜测写一份值出来** —— 限位与运动包络是安全参数。",
              file=sys.stderr)
        return 2

    try:
        joints = load_joints(root)
    except ValueError as exc:
        print(f"✗ 解析固件参数表失败：{exc}", file=sys.stderr)
        return 2
    print(f"固件参数表：{root}")

    output_dir = Path(args.output_dir)
    targets = {
        output_dir / "joint_limits.yaml": render_joint_limits(joints),
        output_dir / "initial_positions.yaml": render_initial_positions(),
    }

    stale: List[str] = []
    for path, content in targets.items():
        if args.check:
            existing = path.read_text(encoding="utf-8") if path.exists() else ""
            if existing != content:
                stale.append(str(path))
            continue
        path.write_text(content, encoding="utf-8")
        print(f"✓ 已写入 {path}")

    if args.check:
        if stale:
            print("✗ 以下文件与固件参数表不同步：")
            for path in stale:
                print(f"    {path}")
            print("  请运行：python3 tools/gen_from_firmware_config.py")
            return 1
        print("✓ joint_limits.yaml / initial_positions.yaml 与固件参数表一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
