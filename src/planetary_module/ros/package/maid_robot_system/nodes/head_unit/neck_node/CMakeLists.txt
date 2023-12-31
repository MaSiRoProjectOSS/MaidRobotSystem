# [node] neck_node

set(NODE_NAME neck_node)

add_definitions(
  -DNODE_NAME="${NODE_NAME}"
)

add_executable(${NODE_NAME}
  src/main.cpp
  src/ros/node_implement.cpp
  src/models/model_implement.cpp
)

target_include_directories(${NODE_NAME}
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
  ${maid_robot_system_interfaces_INCLUDE_DIRS}
PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

ament_target_dependencies(${NODE_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  maid_robot_system_interfaces
)

install(TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
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
