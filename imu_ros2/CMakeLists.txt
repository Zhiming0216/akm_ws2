cmake_minimum_required(VERSION 3.8)
project(imu_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(lpms_nav3_node
  src/lpms_nav3_node.cpp
)

add_executable(quat_to_euler_node
  src/quat_to_euler_node.cpp
)
add_executable(imudata_rad_to_deg_node
  src/imudata_rad_to_deg_node.cpp
)

target_link_libraries(lpms_nav3_node LpmsIG1_OpenSourceLib)

ament_target_dependencies(lpms_nav3_node rclcpp std_msgs std_srvs sensor_msgs)
ament_target_dependencies(quat_to_euler_node rclcpp sensor_msgs tf2)
ament_target_dependencies(imudata_rad_to_deg_node rclcpp sensor_msgs tf2)

install(TARGETS lpms_nav3_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS quat_to_euler_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS imudata_rad_to_deg_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
