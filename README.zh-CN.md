# litearm-moveit2

**LiteArm 机械臂**系列在 ROS 2 上的 MoveIt 2 集成。

[English](README.md) · **简体中文**

> **状态：** `litearm_moveit_config` 包已落地。目前还没有自动化测试；快速上手里的
> 验收探针就是它的检查手段。

## 快速上手

以下命令在工作区根目录执行 —— 同时含 `src/` 与 `install/` 的那一层：

```bash
colcon build --packages-select litearm_moveit_config
source install/setup.bash
ros2 launch litearm_moveit_config demo.launch.py
```

`demo.launch.py` 用 `start_control:=false` 起 MoveIt：可以在 RViz 里规划，而 Execute 需要
控制栈，控制栈又需要 USB 另一头真的接着机械臂。本部署**没有 dry-run**：硬件组件经
litearm C++ SDK 直接与真板子通信。

⚠ 这套栈是**单实例**的，而让它单实例的是**串口**：组件独占机械臂的 USB CDC 链路，SDK 对其加
独占 `flock`，于是第二个控制栈打不开串口、它的硬件组件 configure 失败。此时 `move_group`
照常起来、照常能规划 —— 规划不需要硬件，执行才需要 —— 所以症状是「控制器不见了」，而不是
「launch 挂了」。先停掉另一个栈。

RViz 里 MotionPlanning 面板的规划组是 `litearm_arm`。拖动交互式标记，或选一个 SRDF
命名状态再点 Plan & Execute：

| 命名状态 | 位形 |
| --- | --- |
| `zero` | 七个关节全零 —— 固件零点，上电后的构型 |
| `ready` | 肘部抬起的舒展构型；用它作规划起点，`zero` 在奇异附近，笛卡尔规划在那里会失败 |

无显示环境改用验收探针：它起同一套栈、规划到 `ready`、打印检查项，然后收栈：

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1             # 见下方说明
ros2 run litearm_moveit_config acceptance_moveit.sh                    # 只规划，不起控制栈
ros2 run litearm_moveit_config acceptance_moveit.sh --execute --real   # 真机：规划并执行
```

⚠ 先导出这两个变量。脚本只在它们**未设置**时才用默认值，因此如果当前 shell 里已经有
别的值（`ROS_LOCALHOST_ONLY=0` 很常见），探针会落在与它刚起的栈**不同的发现分区**里，
于是报 `/plan_kinematic_path 不可用（move_group 未就绪？）` —— 而 move_group 其实已经
就绪，它指向的日志里也一条错误都没有。

### 真机

```bash
ros2 launch litearm_moveit_config litearm_moveit.launch.py
```

⚠ 这条会真的驱动机械臂。板子上电、USB 接好、license 已激活，并确认机械臂已支撑、
急停可触达。

控制栈已在别处运行（例如真机调试时手动起的）时加 `start_control:=false`，避免两个
`controller_manager` 抢同一批命令接口。`port` 留空（默认）按 VID:PID `1d50:606f`
自动发现板子；接多块板卡时用 `port:=/dev/ttyACM1` 指定。

这条 launch 还会在配置硬件时**清掉机械臂上锁存的故障**（`clear_faults`，默认开）。
固件会把急停或关节故障锁存住，并在 RESET 之前一律拒绝 ENABLE —— 没有这一步，故障之后
每次启动都要先单独跑一趟 `litearm_driver`。清故障发生在链路已连、尚未使能、尚未下发任何
帧的时候，并且会打一条 WARNING；如果成因还在，固件会重新置位，ENABLE 会带着原因报错。
想让锁存故障**拦住启动**（例如刚出过事故、想先看清机械臂状态）就加 `clear_faults:=false`。

### 从另一个终端连上这套栈

两个 launch 都把 ROS 域钉在 42 并开启仅本机发现，因此默认终端里什么都看不到，需要先
导出同样的变量：

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1
```

用 `ros_domain_id:=0 ros_localhost_only:=false` 可以加入默认图。共享网络下**不要**这么
做：别家的 `move_group` 可能接管你的目标，用一套它没有的控制器把它跑失败，现场看起来
像硬件故障。

如果 `ros2` 命令仍然什么都看不到，加 `--no-daemon` 再试一次：命令行默认从缓存守护进程
取图，而那个守护进程可能还停在导出变量之前的发现设置上。

### 下一步：MoveIt 2 教程

以下都指向 MoveIt 2 官方教程的 Humble 版，与本栈的目标发行版一致：

| 教程 | 讲什么 |
| --- | --- |
| [MoveIt Quickstart in RViz](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) | 本 launch 打开的就是这个面板：规划组、规划路径、交互式标记 |
| [MoveIt Setup Assistant](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html) | `config/` 下的 SRDF、关节限位、运动学文件是怎么生成的 |
| [URDF and SRDF](https://moveit.picknik.ai/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html) | 机器人描述与本包语义描述的分工 |
| [Kinematics Configuration](https://moveit.picknik.ai/humble/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html) | `config/kinematics.yaml`，IK 求解器的选择与预算 |
| [Low Level Controllers](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html) | MoveIt 如何把轨迹交给 ros2_control —— `config/moveit_controllers.yaml` 实现的正是它 |
| [Move Group C++ Interface](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html) | 不经过 RViz，在自己的节点里规划并执行 |
| [MoveIt Task Constructor](https://moveit.picknik.ai/humble/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html) | 多阶段任务（如抓取放置），也就是 `litearm_manipulation` 的方向 |

教程总览：[tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)；
示例总览：[examples](https://moveit.picknik.ai/humble/doc/examples/examples.html)。

## 适用范围

| | |
| --- | --- |
| 产品 | LiteArm 机械臂系列 |
| 本仓库定位 | ROS 2 上的 MoveIt 2 集成 |
| ROS 2 发行版 | Humble Hawksbill |
| 状态 | 活跃 —— `litearm_moveit_config` 可用 |

## 包内容

| 包 | 作用 |
| --- | --- |
| `litearm_moveit_config` | litearm 七轴臂的 MoveIt 2 配置：SRDF 与碰撞矩阵、KDL 运动学、由固件参数表派生的关节限位、OMPL 规划流水线、`moveit_controllers`、RViz 布局、launch 文件，以及验收探针 |

其中两点在各自文件头部有详细说明，这里只给结论：

- **IK 求解器是 KDL（MoveIt 自带）。** 它随 moveit_core 一起装好，工作区不需要额外的
  插件包。KDL 会漏掉真实存在的解 —— 本臂实测：4 个种子各试一遍，705 ms 内可能全部无解，
  而这些位姿 SNS-IK 解得出来 —— 所以当规划表现为**间歇性失败**而不是几何不可达时，
  换回 SNS-IK：配方、插件名陷阱（必须用 `::` 形式）与命中率对比都写在
  `config/kinematics.yaml` 里。
- **Pilz 流水线默认不加载。** 本包只用 OMPL 规划。`config/pilz_cartesian_limits.yaml`
  的存在只是为了让日后启用 Pilz 时不必再补文件。

## 会自己生成的配置

`config/` 下有三个文件是派生出来的，不是手写的。它们各自的文件头都写着「请勿手工维护」：

| 文件 | 由谁生成 | 什么时候重跑 |
| --- | --- | --- |
| `config/joint_limits.yaml` | `tools/gen_from_firmware_config.py <litearm-stm32>` | 固件参数表变了 |
| `config/initial_positions.yaml` | 同上 | 同上 |
| SRDF 里的 `disable_collisions` 区块 | `tools/compute_collision_matrix.py` | URDF 或网格变了 |

关节限位来自固件，且刻意比 URDF 收窄 1°：MoveIt 必须规划在固件运行期强制执行的安全包络
之内，否则会出现「规划成功、执行被拒」。

碰撞矩阵由真实网格采样得出：只有当一对 link 结构相邻、或在**每一个**采样位形下都接触
时才禁用检查。仅仅「会接近」的 link 对保留检查 —— 那些正是 MoveIt 应当避开的区域。

## 相关仓库

| 仓库 | 作用 |
| --- | --- |
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK |
| [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) | C++ SDK |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | 产品文档 |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | ROS 2 驱动 |

## 仓库规范

本仓库遵循 NEXFORM ROBOTICS 的共享仓库规范：[AGENTS.md](AGENTS.md) 里的代理作业规则、
Conventional Commits，以及每次合入 `main` 时自动执行的 semantic-release 版本管理。

## 许可证

版权所有 © 2026 NEXFORM ROBOTICS。以 [Apache License 2.0](LICENSE) 发布。
