# [node] eye_node

set(NODE_NAME eye_node)

add_definitions(
  -DNODE_NAME="${NODE_NAME}"
)
set(CMAKE_AUTOMOC ON)
# Packages
if(DEFINED ENV{MRS_BUILD_LIB_QT6})
  set(BUILD_LIB_QT6 $ENV{MRS_BUILD_LIB_QT6})
  if(BUILD_LIB_QT6)

    find_package(Qt6 REQUIRED COMPONENTS OpenGL)
    set(QT_LIBRARIES Qt6::Widgets)
    set(QT_LIBRARIES_GL Qt6::OpenGL)
    set(QTCORE_INCLUDE_DIRS ${Qt6Core_INCLUDE_DIRS})
    set(QTWIDGETS_INCLUDE_DIRS ${Qt6Widgets_INCLUDE_DIRS})
  endif()
else()
  set(BUILD_LIB_QT6 OFF)
endif()

if(DEFINED ENV{MRS_BUILD_LIB_QT5})
  set(BUILD_LIB_QT5 $ENV{MRS_BUILD_LIB_QT5})
  if(BUILD_LIB_QT5)
    find_package(Qt5 REQUIRED COMPONENTS
      Core
      OpenGL
      Widgets
      Gui
    )
    set(QT_LIBRARIES Qt5::Widgets)
    set(QT_LIBRARIES_GL Qt5::OpenGL)
    set(QTCORE_INCLUDE_DIRS ${Qt5Core_INCLUDE_DIRS})
    set(QTWIDGETS_INCLUDE_DIRS ${Qt5Widgets_INCLUDE_DIRS})
    add_definitions(-DQT_NO_KEYWORDS)

  endif()
else()
  set(BUILD_LIB_QT5 OFF)
endif()

find_package(nlohmann_json REQUIRED)

# source code

add_executable(${NODE_NAME}
  src/main.cpp
  src/ros/node_implement.cpp
  src/models/logger/log_store.cpp

  src/models/widgets/parts/parts_eyeball.cpp
  src/models/widgets/parts/parts_eyelid.cpp

  src/models/model_public_setter_getter.cpp
  src/models/model_public.cpp
  src/models/model_param.cpp

  src/models/widgets/eye_widget_common.cpp
  src/models/widgets/eye_widget_gl.cpp
  src/models/widgets/eye_widget_log.cpp
  src/models/widgets/eye_widget_paint.cpp
  src/models/widgets/eye_widget_public.cpp
  src/models/widgets/eye_widget_loader.cpp


)

target_include_directories(${NODE_NAME}
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include>
  ${maid_robot_system_interfaces_INCLUDE_DIRS}
  ${QTCORE_INCLUDE_DIRS}
  ${QTWIDGETS_INCLUDE_DIRS}
PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

ament_target_dependencies(${NODE_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  maid_robot_system_interfaces
)

target_link_libraries(${NODE_NAME}
    ${QT_LIBRARIES}
    ${QT_LIBRARIES_GL}
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
