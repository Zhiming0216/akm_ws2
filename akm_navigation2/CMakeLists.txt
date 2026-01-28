cmake_minimum_required(VERSION 3.8)
project(akm_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
# nav2_bringup 主要是运行时依赖，编译时其实不需要它，
# 但为了保持 package.xml 一致性，保留也无妨
find_package(nav2_bringup REQUIRED) 

# ========================================================
# 2. 核心修改：添加安装规则 (Install Rules)
# ========================================================
# 这告诉编译器：把我的文件夹拷贝到 install/share/akm_navigation2 下

# 安装 launch 文件夹 (存放启动脚本)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 params 文件夹 (存放 nav2_params.yaml)
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

# 安装 maps 文件夹 (存放 .pgm 和 .yaml 地图)
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# 安装 rviz 文件夹 (如果你有保存 rviz 配置的话)
# 如果你还没创建 rviz 文件夹，可以先注释掉下面这行，否则编译会报错
# install(DIRECTORY rviz
#   DESTINATION share/${PROJECT_NAME}
# )

# ========================================================
# 3. 测试部分 (保持默认即可)
# ========================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()