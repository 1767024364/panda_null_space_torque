cmake_minimum_required(VERSION 3.8)
project(panda_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)

# 编译.cpp并生成.so（将控制器实现编译成共享库）
add_library(${PROJECT_NAME} SHARED src/null_space_controller.cpp
                                   src/pose_null_space_controller_torque.cpp)

# 让库找到自己的头文件
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# 把依赖关联到.so（把依赖关联到该库）
ament_target_dependencies(${PROJECT_NAME} 
    controller_interface 
    pluginlib
    pinocchio
    rclcpp
    hardware_interface
  )




# 注册插件 XML
# 这个宏会把 XML 安装到 share/panda_controllers，
# 并登记到 ament index，供 controller_manager 查找
pluginlib_export_plugin_description_file(controller_interface panda_controllers_plugin.xml)



# 安装头文件、共享文件、脚本
install(DIRECTORY include/ DESTINATION include)




# 安装.so（安装共享库）
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



# 导出库与依赖，供其他 ROS 2 包使用
# 
# 



# 必须放在最后：生成并导出 ament 包信息
ament_package()

# ------------------------------------以下是 CMakeList.txt 的编写步骤------------------------------------


# find_package              找到依赖
# add_library               编译.cpp并生成.so
# target_include_directories 让编译器找到自己的头文件
# ament_target_dependencies 把依赖关联到.so
# pluginlib_export...       让pluginlib发现插件XML
# install(TARGETS...)       安装.so
# install(DIRECTORY...)     安装头文件
# ament_package             注册ROS 2包



#[[
cmake_minimum_required(VERSION 3.8)
project(panda_controllers)

# 1. 编译选项
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 查找本包实际使用的依赖（******** 必须配置 ********）
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 3. 将控制器实现编译成共享库 libpanda_controllers.so（******** 必须配置 ********）
add_library(${PROJECT_NAME} SHARED
  src/null_space_controller.cpp
)

# 4. 让本库找到自己的头文件（******** 必须配置 ********）
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 5. 将 ROS 2 依赖关联到该库
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  pluginlib
)

# 6. 注册插件 XML
# 这个宏会把 XML 安装到 share/panda_controllers，
# 并登记到 ament index，供 controller_manager 查找。
pluginlib_export_plugin_description_file(
  controller_interface
  panda_controllers_plugin.xml
)

# 7. 安装共享库
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 8. 安装头文件、共享文件、脚本
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/joint_state_listener.py DESTINATION lib/${PROJECT_NAME})

# 9. 导出库与依赖，供其他 ROS 2 包使用
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface
  pluginlib
)

# 10. 必须放在最后：生成并导出 ament 包信息
ament_package()
]]