cmake_minimum_required(VERSION 3.8)
project(ired_dwb_critics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(ired_dwb_critics SHARED src/goal_yaw.cpp)

target_include_directories(
  ired_dwb_critics PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ired_dwb_critics
  angles
  dwb_core
  dwb_msgs
  geometry_msgs
  nav_2d_msgs
  pluginlib)

pluginlib_export_plugin_description_file(dwb_core goal_yaw_plugin.xml)

install(
  TARGETS ired_dwb_critics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

install(FILES goal_yaw_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
