cmake_minimum_required(VERSION 3.8)
project(panda_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)

# 编译源文件

# 让库找到自己的头文件

# 把依赖关联到.so（把依赖关联到该库）

# 注册插件 XML
# 这个宏会把 XML 安装到 share/panda_controllers，
# 并登记到 ament index，供 controller_manager 查找

# 安装头文件、共享文件、脚本
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# 安装.so（安装共享库）



ament_package()
