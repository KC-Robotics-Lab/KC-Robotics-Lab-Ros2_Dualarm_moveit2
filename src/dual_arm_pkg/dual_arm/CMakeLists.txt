cmake_minimum_required(VERSION 3.8)
project(dual_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ur_pick_and_place_msgs REQUIRED)
find_package(rbpodo_msgs REQUIRED)
find_package(gripper_srv REQUIRED)
find_package(dual_arm_msg REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(inspire_hand_interface REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(yaml-cpp QUIET CONFIG)
if(NOT yaml-cpp_FOUND)
  find_package(yaml-cpp REQUIRED)  # Ubuntu libyaml-cpp-dev의 Find 모듈
endif()

include_directories(include)

add_executable(hand_gesture src/hand_gesture.cpp)
ament_target_dependencies(hand_gesture rclcpp moveit_ros_planning_interface ur_pick_and_place_msgs rbpodo_msgs dual_arm_msg inspire_hand_interface)

# add_executable(bimanual_pnp src/bimanual_pnp.cpp) // 원래 코드
# ament_target_dependencies(bimanual_pnp rclcpp moveit_ros_planning_interface ur_pick_and_place_msgs rbpodo_msgs dual_arm_msg inspire_hand_interface control_msgs)

# add_executable(bimanual_pnp src/bimanual_async.cpp)
# ament_target_dependencies(bimanual_pnp rclcpp moveit_ros_planning_interface ur_pick_and_place_msgs rbpodo_msgs dual_arm_msg inspire_hand_interface control_msgs std_msgs)

add_executable(bimanual_pnp src/bimanual_seq_async.cpp)
ament_target_dependencies(bimanual_pnp rclcpp moveit_ros_planning_interface ur_pick_and_place_msgs rbpodo_msgs dual_arm_msg inspire_hand_interface control_msgs std_msgs)

add_executable(dual_arm src/dual_arm.cpp)
ament_target_dependencies(dual_arm rclcpp moveit_ros_planning_interface ur_pick_and_place_msgs rbpodo_msgs dual_arm_msg)

add_executable(bimanual_service src/bimanual_service.cpp)
ament_target_dependencies(bimanual_service 
  rclcpp 
  moveit_ros_planning_interface 
  ur_pick_and_place_msgs 
  rbpodo_msgs 
  dual_arm_msg 
  rclcpp_action
  control_msgs
  trajectory_msgs)

add_library(gripper_client_lib src/gripper_client.cpp)
ament_target_dependencies(gripper_client_lib rclcpp gripper_srv)

target_link_libraries(bimanual_pnp gripper_client_lib)
target_link_libraries(dual_arm gripper_client_lib)
target_link_libraries(bimanual_service gripper_client_lib)


install(TARGETS
  bimanual_service
  hand_gesture
  bimanual_pnp
  dual_arm
  gripper_client_lib
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/kc_ros_zmq_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf rviz srdf config scripts
  DESTINATION share/${PROJECT_NAME}
)

# plain signature만 사용 (PUBLIC/PRIVATE 금지)
if(TARGET YAML::yaml-cpp)
  target_link_libraries(bimanual_pnp YAML::yaml-cpp)
elseif(TARGET yaml-cpp)
  target_link_libraries(bimanual_pnp yaml-cpp)
else()
  message(FATAL_ERROR "Could not find a yaml-cpp CMake target to link against.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
