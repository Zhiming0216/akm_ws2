cmake_minimum_required(VERSION 3.8)
project(ackerman_chassis2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ID501.srv"
  "msg/ID502.msg"
  "msg/ID503.msg"
)

# =========================================================
# 底盘驱动
# =========================================================
add_executable(ackerman_chassis_node2 src/ackerman_chassis_node2.cpp)

ament_target_dependencies(ackerman_chassis_node2
  rclcpp
  geometry_msgs
  std_srvs
)

# 链接自定义消息类型支持
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ackerman_chassis_node2 "${cpp_typesupport_target}")

# =========================================================
# 安装规则
# =========================================================
ament_export_dependencies(rosidl_default_runtime)

# 安装两个可执行文件
install(TARGETS
  ackerman_chassis_node2
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装参数文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()