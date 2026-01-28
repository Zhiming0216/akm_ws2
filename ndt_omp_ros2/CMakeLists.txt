cmake_minimum_required(VERSION 3.5)
project(ndt_omp_ros2)

# 基础设置
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_BUILD_TYPE "RELEASE")

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED) # 建议加上这个

# PCL 查找 (去掉版本号限制，提高兼容性)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

# OpenMP 查找
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# 2. 包含路径
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 3. 编译库 (关键修改：显式声明 SHARED 生成 .so 文件)
add_library(ndt_omp SHARED
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

# 链接依赖
target_link_libraries(ndt_omp
  ${PCL_LIBRARIES}
)

# 4. 编译测试程序 (align)
add_executable(align apps/align.cpp)
target_link_libraries(align
  ndt_omp
  ${PCL_LIBRARIES}
)
ament_target_dependencies(align
  rclcpp
  std_msgs
)

# 5. 安装规则 (🚨 核心修复：这里必须要把库文件 ndt_omp 装进去)
install(TARGETS 
  ndt_omp    # <--- 原文件漏掉了这个！
  align
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 6. 导出依赖 (告诉别的包：我有 ndt_omp 这个库)
ament_export_include_directories(include)
ament_export_libraries(ndt_omp)       # <--- 加上这一行
ament_export_dependencies(rclcpp PCL) # <--- 加上 PCL

ament_package()