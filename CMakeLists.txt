cmake_minimum_required(VERSION 3.8)
project(map_edit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Quick Qml QuickWidgets)
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


# MOC 文件（用于 QObject 继承类）
qt5_wrap_cpp(MOC_FILES
  include/map_edit/map_edit_panel.h
  include/map_edit/map_eraser_tool.h
  include/map_edit/tool_manager.h

)

# 源文件
set(SRC_FILES
  src/map_edit_panel.cpp
  src/tool_manager.cpp
  src/map_eraser_tool.cpp
)

# 头文件包含路径
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# 构建共享库（rviz 插件要求）
add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${MOC_FILES}
)

# 链接 Qt 和 ROS 所需库
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::Quick
  Qt5::Qml
  Qt5::QuickWidgets
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  ament_index_cpp
  nav_msgs
  tf2_ros
  rviz_rendering
  cv_bridge
  tf2
  visualization_msgs
)

# 安装共享库
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)


# 安装插件描述文件（pluginlib）
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
