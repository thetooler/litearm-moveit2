cmake_minimum_required(VERSION 3.16)
project(litearm_moveit_config)

find_package(ament_cmake REQUIRED)

# 本包是纯配置包（SRDF / yaml / launch / rviz），不含编译产物。
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 现场验收探针（需要已启动的 move_group）
install(PROGRAMS
  scripts/moveit_probe.py
  scripts/acceptance_moveit.sh
  DESTINATION lib/${PROJECT_NAME}
)

# 工具脚本：碰撞矩阵生成、从固件参数表派生关节限位。
# 装到 lib/${PROJECT_NAME} 下，系统提示里给出的调用路径才是有效的。
install(PROGRAMS
  tools/compute_collision_matrix.py
  tools/gen_from_firmware_config.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
