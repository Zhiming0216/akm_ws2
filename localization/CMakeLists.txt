cmake_minimum_required(VERSION 3.8)
project(localization)

# ============================================
# 1. 基础编译选项
# ============================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 强制使用 C++17 (PCL 和 ROS2 Humble+ 的标准)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================
# 2. 查找依赖包
# ============================================
find_package(ament_cmake REQUIRED)

# ROS 基础包
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# TF 变换相关
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# PCL 转换与工具 (关键！)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED) # <--- 必须有这个才能用 transformPointCloud

# 查找系统级 PCL
find_package(PCL REQUIRED)

# 查找 NDT_OMP 加速库
# 注意：根据你的环境，包名可能是 'ndt_omp' 或 'ndt_omp_ros2'
# 如果报错找不到，请尝试把这里改为 find_package(ndt_omp REQUIRED)
find_package(ndt_omp_ros2 REQUIRED) 

# 查找 OpenMP (NDT_OMP 必须依赖这个)
find_package(OpenMP REQUIRED)

# ============================================
# 3. 配置 PCL 环境
# ============================================
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# ============================================
# 4. 定义可执行文件
# ============================================
add_executable(ndt_localization_node src/localization_node.cpp)

# ============================================
# 5. 包含头文件路径
# ============================================
target_include_directories(ndt_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
  ${ndt_omp_ros2_INCLUDE_DIRS} # 如果上面改了包名，这里也要改
)

# ============================================
# 6. 链接库 (Link Libraries)
# ============================================
target_link_libraries(ndt_localization_node
  ${PCL_LIBRARIES}
  ${ndt_omp_ros2_LIBRARIES}    # NDT 加速库
  OpenMP::OpenMP_CXX           # 启用多线程加速
)

# ============================================
# 7. 链接 ROS 依赖 (Ament Dependencies)
# ============================================
ament_target_dependencies(ndt_localization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  pcl_ros  # <--- 确保 pcl_ros 被链接
)

# ============================================
# 8. 安装规则
# ============================================
# 安装可执行文件
install(TARGETS ndt_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

# # 安装 Launch 文件和 Param 文件 (如果有)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*"
)

# 安装地图文件夹 (如果你想把地图打包进 workspace)
install(DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.pcd"
)

ament_package()