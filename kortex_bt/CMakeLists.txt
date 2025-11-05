cmake_minimum_required(VERSION 3.8)
project(kortex_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(LibXml2 REQUIRED)

add_executable(bt_runner
  src/bt.cpp
  src/mission_tcp.cpp
  src/xml_validation.cpp
  src/actions/check_value.cpp
  src/actions/assert_true.cpp
)

target_include_directories(bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_runner
  rclcpp
  behaviortree_ros2
)

target_link_libraries(bt_runner
  LibXml2::LibXml2
)

install(TARGETS bt_runner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY schemas/
  DESTINATION share/${PROJECT_NAME}/schemas
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
