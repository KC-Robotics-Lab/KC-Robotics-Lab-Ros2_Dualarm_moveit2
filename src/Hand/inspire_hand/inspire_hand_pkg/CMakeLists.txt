cmake_minimum_required(VERSION 3.8)
project(inspire_hand_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_PREFIX_PATH /serial)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(inspire_hand_interface REQUIRED)
find_package(serial REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(Hand_Control src/Hand_Control_LowData.cpp)
ament_target_dependencies(Hand_Control rclcpp std_msgs std_srvs rclcpp_action serial inspire_hand_interface)

add_executable(Tactile_data_pub src/Tactile_data_pub.cpp)
ament_target_dependencies(Tactile_data_pub rclcpp std_msgs std_srvs serial inspire_hand_interface)

add_executable(get_posact_client src/get_posact_client.cpp)
ament_target_dependencies(get_posact_client rclcpp std_msgs inspire_hand_interface)

add_executable(action_angle src/action.cpp)
ament_target_dependencies(action_angle rclcpp std_msgs rclcpp_action inspire_hand_interface serial)

# add_executable(Hand_Control src/Hand_Control.cpp)
# ament_target_dependencies(Hand_Control rclcpp std_msgs std_srvs serial inspire_hand_interface)

# add_executable(handparam src/get_handparam_pub.cpp)
# ament_target_dependencies(handparam rclcpp std_msgs inspire_hand_interface serial)

# add_executable(pos_sub src/sub_pos_act.cpp)
# ament_target_dependencies(pos_sub rclcpp std_msgs inspire_hand_interface)

# add_executable(service_hand src/service_hand.cpp)
# ament_target_dependencies(service_hand rclcpp std_msgs inspire_hand_interface serial)

# add_executable(get_tactile_little src/get_tactile_little.cpp)
# ament_target_dependencies(get_tactile_little rclcpp std_msgs serial)
# add_executable(get_tactile_ring src/get_tactile_ring.cpp)
# ament_target_dependencies(get_tactile_ring rclcpp std_msgs serial)
# add_executable(get_tactile_middle src/get_tactile_middle.cpp)
# ament_target_dependencies(get_tactile_middle rclcpp std_msgs serial)
# add_executable(get_tactile_index src/get_tactile_index.cpp)
# ament_target_dependencies(get_tactile_index rclcpp std_msgs serial)
# add_executable(get_tactile_thumb src/get_tactile_thumb.cpp)
# ament_target_dependencies(get_tactile_thumb rclcpp std_msgs serial)

install(TARGETS
  action_angle
  Hand_Control
  Tactile_data_pub
  get_posact_client
  # handparam
  # pos_sub
  # service_hand 
  # get_tactile_little
  # get_tactile_ring
  # get_tactile_middle
  # get_tactile_index
  # get_tactile_thumb
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

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
