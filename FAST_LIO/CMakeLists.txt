cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ================= 编译选项优化 =================
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 开启 O3 优化，这对 FastLIO 性能至关重要
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# ================= CPU 架构检测与多线程配置 =================
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# ================= 依赖包查找 =================
find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS 2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 第三方库
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# ================= 消息生成 =================
set(msg_files
  "msg/Pose6D.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

# 定义所有依赖列表
set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

# ==========================================================
#  🔥 核心修改 1: 定义建图节点 (fastlio_mapping)
#  只包含 laserMapping.cpp
# ==========================================================
add_executable(fastlio_mapping 
  src/laserMapping.cpp 
  src/preprocess.cpp 
  include/ikd-Tree/ikd_Tree.cpp
  # 如果你需要回环检测，请取消下面两行的注释：
  # src/Scancontext.cpp
  # src/loop_closure.cpp
)

target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

target_link_libraries(fastlio_mapping 
  ${PCL_LIBRARIES} 
  ${PYTHON_LIBRARIES} 
  Eigen3::Eigen
  # 如果你需要回环检测，且安装了GTSAM，请取消下面这行的注释：
  # gtsam
)

# ==========================================================
#  🔥 核心修改 2: 定义纯里程计节点 (fastlio_odometry)
#  只包含 laserOdometry.cpp
# ==========================================================
add_executable(fastlio_odometry 
  src/laserOdometry.cpp 
  src/preprocess.cpp 
  include/ikd-Tree/ikd_Tree.cpp
)

target_include_directories(fastlio_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

target_link_libraries(fastlio_odometry 
  ${PCL_LIBRARIES} 
  ${PYTHON_LIBRARIES} 
  Eigen3::Eigen
)

# ================= 处理 ROS 2 版本兼容性 =================
list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")

if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  rosidl_target_interfaces(fastlio_mapping ${PROJECT_NAME} "rosidl_typesupport_cpp")
  rosidl_target_interfaces(fastlio_odometry ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping "${cpp_typesupport_target}")
  target_link_libraries(fastlio_odometry "${cpp_typesupport_target}")
endif()

# 链接 ROS 依赖
ament_target_dependencies(fastlio_mapping ${dependencies})
ament_target_dependencies(fastlio_odometry ${dependencies})

# ================= 安装规则 =================
install(TARGETS 
  fastlio_mapping
  fastlio_odometry  # 别忘了安装这个新节点
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()