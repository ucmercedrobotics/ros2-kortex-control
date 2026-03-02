cmake_minimum_required(VERSION 3.8)
project(leaf_grasping_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(kortex_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)


# Define the C++ executables
add_executable(arm_commander src/arm_commander.cpp)
add_executable(target_manager src/target_manager.cpp)

# Add include directories for your executables to find the action headers
target_include_directories(arm_commander PUBLIC
  $<BUILD_INTERFACE:${kortex_interfaces_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>)
target_include_directories(target_manager PUBLIC
  $<BUILD_INTERFACE:${kortex_interfaces_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(arm_commander
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  geometry_msgs
  tf2_ros
  moveit_msgs
  shape_msgs
  kortex_interfaces
  custom_interfaces
)

ament_target_dependencies(target_manager
  rclcpp
  rclcpp_action
  kortex_interfaces
)

# Install both C++ executables
install(TARGETS
  arm_commander
  target_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  test/test_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  resources/
  DESTINATION share/${PROJECT_NAME}/resources
)


install(DIRECTORY

  launch

  DESTINATION share/${PROJECT_NAME}

)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
