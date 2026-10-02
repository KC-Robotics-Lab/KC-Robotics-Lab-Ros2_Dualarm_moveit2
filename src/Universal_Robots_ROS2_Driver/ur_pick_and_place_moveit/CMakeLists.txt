cmake_minimum_required(VERSION 3.8)
project(ur_pick_and_place_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gripper_srv REQUIRED)
find_package(ur_pick_and_place_msgs REQUIRED)

include_directories(include)

add_executable(ur_pick_and_place src/ur_pick_and_place.cpp)
ament_target_dependencies(ur_pick_and_place rclcpp moveit_ros_planning_interface)

add_executable(ur_pick src/ur_pick.cpp)
ament_target_dependencies(ur_pick rclcpp moveit_ros_planning_interface)

# add_executable(picknplace src/picknplace.cpp)
# ament_target_dependencies(picknplace rclcpp moveit_ros_planning_interface ur_pick_and_place_msgs)

add_executable(move_ur src/move_ur.cpp)
ament_target_dependencies(move_ur rclcpp moveit_ros_planning_interface ur_pick_and_place_msgs)

add_library(gripper_client_lib src/gripper_client.cpp)
ament_target_dependencies(gripper_client_lib rclcpp gripper_srv)

target_link_libraries(ur_pick_and_place gripper_client_lib)
target_link_libraries(ur_pick gripper_client_lib)
target_link_libraries(move_ur gripper_client_lib)

install(TARGETS
  ur_pick_and_place
  ur_pick
  # picknplace
  move_ur
  gripper_client_lib
  DESTINATION lib/${PROJECT_NAME})

  # Install launch files.
install(DIRECTORY
  launch config srdf
  DESTINATION share/${PROJECT_NAME}/
)



# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/Gohome.srv"
#   DEPENDENCIES
# )
# ament_export_dependencies(rosidl_default_runtime)

# include_directories(
#   include
# )

# add_executable(ur_pick_and_place src/ur_pick_and_place.cpp)
# target_include_directories(ur_pick_and_place PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(ur_pick_and_place PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# add_executable(ur_pick src/ur_pick.cpp)
# target_include_directories(ur_pick PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(ur_pick PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# add_executable(pickNplace src/pickNplace.cpp)
# target_include_directories(pickNplace PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
#   $<INSTALL_INTERFACE:include>
# )

# add_library(gripper_client_lib
#   src/gripper_client.cpp
# )

# ament_target_dependencies(gripper_client_lib rclcpp gripper_srv)
# target_include_directories(gripper_client_lib PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# target_link_libraries(ur_pick_and_place gripper_client_lib) # Link the library
# target_link_libraries(ur_pick gripper_client_lib) # Link the library
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(pickNplace ${cpp_typesupport_target})

# install(TARGETS
#   ur_pick_and_place
#   ur_pick
#   pickNplace
#   gripper_client_lib
#   DESTINATION lib/${PROJECT_NAME})

# ament_target_dependencies(
#   ur_pick_and_place
#   "moveit_ros_planning_interface"
#   "rclcpp"
# )

# ament_target_dependencies(
#   ur_pick
#   "moveit_ros_planning_interface"
#   "rclcpp"
# )

# ament_target_dependencies(
#   pickNplace
#   "moveit_ros_planning_interface"
#   "rclcpp"
# )

# # Install launch files.
# install(DIRECTORY
#   launch config srdf
#   DESTINATION share/${PROJECT_NAME}/
# )

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
