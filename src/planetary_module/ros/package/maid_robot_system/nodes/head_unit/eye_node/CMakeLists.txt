# [node] eye_node

set(NODE_NAME eye_node)

add_definitions(
  -DNODE_NAME="${NODE_NAME}"
)

if(DEFINED ENV{MRS_BUILD_LIB_QT6})
  set(BUILD_LIB_QT6 $ENV{MRS_BUILD_LIB_QT6})
  if(BUILD_LIB_QT6)
    find_package(Qt6 REQUIRED COMPONENTS OpenGL)
    target_link_libraries(mytarget PRIVATE Qt6::OpenGL)
  endif()
else()
  set(BUILD_LIB_QT6 OFF)
endif()

if(DEFINED ENV{MRS_BUILD_LIB_QT5})
  set(BUILD_LIB_QT5 $ENV{MRS_BUILD_LIB_QT5})
  if(BUILD_LIB_QT5)
    find_package(Qt5 REQUIRED COMPONENTS OpenGL)
    set(QT_LIBRARIES Qt5::Widgets)
    set(QT_LIBRARIES_GL Qt5::OpenGL)
    add_definitions(-DQT_NO_KEYWORDS)
  endif()
else()
  set(BUILD_LIB_QT5 OFF)
endif()

add_executable(${NODE_NAME}
  src/main.cpp
  src/ros/node_implement.cpp
  src/models/model_public.cpp
  ## modelsの中身は完全に分離してください。
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
