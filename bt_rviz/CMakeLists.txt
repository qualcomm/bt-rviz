cmake_minimum_required(VERSION 3.10)
project(bt_rviz)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bt_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz2 REQUIRED)
find_package(class_loader REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/bt_vis_panel.hpp
)

add_library(bt_vis_panel src/bt_vis_panel.cpp ${MOC_FILES})
target_include_directories(bt_vis_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_vis_panel
  pluginlib
  rviz_common
  bt_msgs
  rviz_rendering
  rviz_ogre_vendor
)
install(TARGETS bt_vis_panel
        EXPORT export_bt_vis_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_bt_vis_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)


set(TINYXML2_INCLUDE_DIR "/usr/include")
set(TINYXML2_LIB "/usr/lib/x86_64-linux-gnu/libtinyxml2.so")

include_directories(
    include
    ${TINYXML2_INCLUDE_DIR}
)


add_library(tree_parser SHARED src/tree_parser.cpp)

target_link_libraries(tree_parser
    ${TINYXML2_LIB}
    rclcpp::rclcpp
)

install(
    TARGETS tree_parser
    EXPORT tree_parser_export
    DESTINATION lib
)


install(
    EXPORT tree_parser_export
    DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(tree_parser_export)

set(SOURCES
  src/bt_monitor.cpp
)

add_library(bt_monitor SHARED ${SOURCES})

ament_target_dependencies(bt_monitor
  rclcpp
  bt_msgs
  behaviortree_cpp_v3
)

install(
    TARGETS bt_monitor
    EXPORT bt_monitor_export
    DESTINATION lib
)


install(
    EXPORT bt_monitor_export
    DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(bt_monitor_export)


install(TARGETS bt_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_target_dependencies(tree_parser rclcpp bt_msgs std_msgs)
ament_package()
