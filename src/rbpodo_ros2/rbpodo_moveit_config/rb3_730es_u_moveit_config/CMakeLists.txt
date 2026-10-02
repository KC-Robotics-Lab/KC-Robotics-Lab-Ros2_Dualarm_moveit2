cmake_minimum_required(VERSION 3.22)
project(rb3_730es_u_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rbpodo_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rbpodo_msgs REQUIRED)

add_executable(rb3_pick_place src/rb3_pick_place.cpp)
ament_target_dependencies(rb3_pick_place rclcpp moveit_ros_planning_interface)

add_executable(move_rb src/move_rb.cpp)
ament_target_dependencies(move_rb rclcpp moveit_ros_planning_interface rbpodo_msgs std_srvs)

# add_library(rb3_730es_u_moveit_config
#   src/move.cpp
# )
# target_compile_features(rb3_730es_u_moveit_config PUBLIC c_std_99 cxx_std_17)
# target_include_directories(rb3_730es_u_moveit_config PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(
#   rclcpp
#   moveit_ros_planning_interface
#   rbpodo_msgs
#   std_srvs
# )

install(TARGETS
  rb3_pick_place
  move_rb
  DESTINATION lib/${PROJECT_NAME})

# ament_target_dependencies(
#   rb3_pick_place
#   "moveit_ros_planning_interface"
#   "rclcpp"
# )

ament_package()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})