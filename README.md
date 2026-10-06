# litearm-moveit2

MoveIt 2 integration on ROS 2 for the **LiteArm robotic manipulator series**.

**English** · [简体中文](README.zh-CN.md)

> **Status:** the `litearm_moveit_config` package has landed. It has no automated
> test suite yet; the acceptance probe in the quickstart is what checks it.

## Quickstart

Run these from the workspace root — the directory that holds both `src/` and
`install/`:

```bash
colcon build --packages-select litearm_moveit_config
source install/setup.bash
ros2 launch litearm_moveit_config demo.launch.py
```

`demo.launch.py` starts MoveIt with `start_control:=false`: the arm plans in RViz,
and Execute needs a control stack, which needs an arm on the other end of the USB
cable. This deployment has no dry-run mode — the hardware component talks to the
real board through the litearm C++ SDK.

⚠ The stack is single-instance, and the serial port is what makes it so: the
component owns the arm's USB CDC link and the SDK takes an exclusive `flock` on
it, so a second control stack fails to open the port and its hardware component
fails to configure. `move_group` still comes up and still plans — planning does
not need the hardware, execution does — so the failure shows up as a missing
controller rather than as a dead launch. Stop the other stack first.

In RViz the MotionPlanning panel's planning group is `litearm_arm`. Drag the
interactive marker, or pick one of the SRDF's named states and press Plan &
Execute:

| Named state | Pose |
| --- | --- |
| `zero` | all seven joints at zero — the firmware zero, and the pose after power-up |
| `ready` | elbow-up configuration; use this as the planning start, `zero` is near a singularity and Cartesian planning fails there |

Without a display, run the acceptance probe instead. It starts the same stack,
plans to `ready`, prints what it checked, and tears the stack down:

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1             # see the note
ros2 run litearm_moveit_config acceptance_moveit.sh                    # plan only, no control stack
ros2 run litearm_moveit_config acceptance_moveit.sh --execute --real   # plan and execute on the arm
```

⚠ Export those two variables first. The script defaults them only when they are
unset, so a shell that already exports a different value — `ROS_LOCALHOST_ONLY=0`
is a common one — puts the probe in a different discovery partition than the
stack it just started. It then reports `/plan_kinematic_path 不可用（move_group
未就绪？）` while move_group is in fact ready and its log holds no errors.

### Real hardware

```bash
ros2 launch litearm_moveit_config litearm_moveit.launch.py
```

⚠ This one moves the arm. Power the board, connect USB, activate the license,
support the arm and keep the e-stop within reach.

Add `start_control:=false` when a control stack is already running, so that two
`controller_manager`s do not fight over the same command interfaces. Empty `port`
(the default) auto-discovers the board by VID:PID `1d50:606f`; pass
`port:=/dev/ttyACM1` when several are attached.

This launch also clears a **latched fault** on the arm while configuring the hardware
(`clear_faults`, on by default). The firmware latches an emergency stop or a joint fault
and then refuses ENABLE until it is reset, so without this a bring-up after a fault needs
a separate `litearm_driver` session first. The reset happens with the link up and nothing
enabled or streamed, and it is logged as a warning; if the cause is still there the
firmware latches the fault again and ENABLE fails with a message that names it. Pass
`clear_faults:=false` when you want a latched fault to stop the launch — after a crash,
for instance, so you can inspect the arm before anything is enabled.

### Reaching the stack from another terminal

Both launch files pin the ROS domain to 42 and enable localhost-only discovery,
so nothing in a plain shell sees the stack until it exports the same settings:

```bash
export ROS_DOMAIN_ID=42; export ROS_LOCALHOST_ONLY=1
```

`ros_domain_id:=0 ros_localhost_only:=false` joins the default graph instead. Do
not do that on a shared network: a stranger's `move_group` can pick up your goal,
fail it against controllers it does not have, and leave the result looking like a
hardware fault.

If a `ros2` command still sees nothing, try it with `--no-daemon`: the CLI
otherwise answers from a cached daemon that may hold the discovery settings from
before your export.

### Where to go next

MoveIt 2 tutorials, Humble edition — the distribution this stack targets:

| Tutorial | Covers |
| --- | --- |
| [MoveIt Quickstart in RViz](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) | the panel this launch opens: planning groups, planned paths, the interactive marker |
| [MoveIt Setup Assistant](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html) | how the SRDF, joint limits and kinematics file under `config/` are produced |
| [URDF and SRDF](https://moveit.picknik.ai/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html) | the split between the robot's description and this package's semantic description |
| [Kinematics Configuration](https://moveit.picknik.ai/humble/doc/examples/kinematics_configuration/kinematics_configuration_tutorial.html) | `config/kinematics.yaml`, the IK solver and its budget |
| [Low Level Controllers](https://moveit.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html) | how MoveIt hands a trajectory to ros2_control — what `config/moveit_controllers.yaml` implements |
| [Move Group C++ Interface](https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html) | planning and executing from your own node instead of RViz |
| [MoveIt Task Constructor](https://moveit.picknik.ai/humble/doc/examples/moveit_task_constructor/moveit_task_constructor_tutorial.html) | multi-stage tasks such as pick and place, the direction `litearm_manipulation` takes |

Full indexes: [tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
and [examples](https://moveit.picknik.ai/humble/doc/examples/examples.html).

## Scope

| | |
| --- | --- |
| Product | LiteArm robotic manipulator series |
| Repository role | MoveIt 2 integration on ROS 2 |
| ROS 2 distribution | Humble Hawksbill |
| Status | Active — `litearm_moveit_config` available |

## Packages

| Package | Role |
| --- | --- |
| `litearm_moveit_config` | MoveIt 2 configuration for the litearm seven-axis arm: SRDF and collision matrix, KDL kinematics, joint limits derived from the firmware parameter table, the OMPL planning pipeline, `moveit_controllers`, RViz layout, launch files, and the acceptance probe |

Two notes on that list, both of which the package's own headers explain at
length:

- **The IK solver is KDL, MoveIt's default.** It ships with MoveIt, so the
  workspace needs no extra package. KDL misses poses that exist — measured on this
  arm, four seeds fail within 705 ms where SNS-IK solves them — so when planning
  looks intermittent rather than geometric, switch to SNS-IK: the recipe, the
  plugin-name trap and the measured hit rates are in `config/kinematics.yaml`.
- **The Pilz pipeline is not loaded.** The package plans with OMPL only.
  `config/pilz_cartesian_limits.yaml` is there so that enabling Pilz needs no new
  file.

## Generated configuration

Three files under `config/` are derived, not written by hand. Each carries a
"do not edit by hand" warning in its own header:

| File | Produced by | Regenerate when |
| --- | --- | --- |
| `config/joint_limits.yaml` | `tools/gen_from_firmware_config.py <litearm-stm32>` | the firmware parameter table changes |
| `config/initial_positions.yaml` | the same | the same |
| the SRDF's `disable_collisions` block | `tools/compute_collision_matrix.py` | the URDF or the meshes change |

The joint limits are the firmware's, deliberately 1° tighter than the URDF's.
MoveIt has to plan inside the envelope the firmware enforces at runtime,
otherwise a plan succeeds and the execution is rejected.

The collision matrix is sampled from the real meshes: the script disables a link
pair only when the pair is structurally adjacent or touches in every sampled
pose. Pairs that merely come close stay checked — those are the ones MoveIt is
supposed to avoid.

## Related repositories

| Repository | Role |
| --- | --- |
| [litearm-python](https://github.com/nexform-tech/litearm-python) | Python SDK |
| [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) | C++ SDK |
| [litearm-docs](https://github.com/nexform-tech/litearm-docs) | Product documentation |
| [litearm-ros2](https://github.com/nexform-tech/litearm-ros2) | ROS 2 driver |

## Repository standards

This repository follows the shared NEXFORM ROBOTICS repository standards: the
agent operating rules in [AGENTS.md](AGENTS.md), Conventional Commits, and
automated semantic-release versioning on every merge to `main`.

## License

Copyright © 2026 NEXFORM ROBOTICS. Licensed under the
[Apache License 2.0](LICENSE).
