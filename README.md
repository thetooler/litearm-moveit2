# litearm-moveit2

MoveIt 2 integration on ROS 2 for the **LiteArm robotic manipulator series**.

> **Status:** the `litearm_moveit_config` package has landed. Package tests and
> CI coverage for it are still to come.

## Scope

| | |
| --- | --- |
| Product | LiteArm robotic manipulator series |
| Repository role | MoveIt 2 integration on ROS 2 |
| Status | Active — `litearm_moveit_config` available |

## Packages

| Package | Role |
| --- | --- |
| `litearm_moveit_config` | MoveIt 2 configuration for the litearm seven-axis arm: SRDF and collision matrix, KDL kinematics, joint limits derived from the firmware parameter table, OMPL and Pilz planning pipelines, `moveit_controllers`, RViz layout and launch files |

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
