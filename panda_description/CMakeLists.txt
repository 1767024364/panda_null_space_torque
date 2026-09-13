cmake_minimum_required(VERSION 3.8)
project(panda_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf mujoco launch DESTINATION share/${PROJECT_NAME})


ament_package()

# 可执行程序       lib/<package_name>/
# C/C++ 库        lib/
# 头文件           include/
# URDF、配置、模型 share/<package_name>/
