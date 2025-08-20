cmake_minimum_required(VERSION 3.8)
project(nav2_reflex_explore)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ---------- Core library ----------
add_library(frontier_core
  src/frontier_core.cpp
)
target_include_directories(frontier_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(frontier_core
  rclcpp
  nav_msgs
)

# ---------- Node ----------
add_executable(reflex_explorer
  src/reflex_explorer_node.cpp
)
target_include_directories(reflex_explorer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(reflex_explorer
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)
target_link_libraries(reflex_explorer frontier_core)

# ---------- Install ----------
install(TARGETS
  frontier_core
  reflex_explorer
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

# ---------- Export for downstream packages ----------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
