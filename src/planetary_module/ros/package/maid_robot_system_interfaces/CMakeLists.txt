cmake_minimum_required(VERSION 3.8)
project(maid_robot_system_interfaces)

# Select build unit
set(BUILD_DEVELOP ON)

if(DEFINED ENV{BUILD_DEVELOP})
  set(BUILD_DEVELOP $ENV{BUILD_DEVELOP})
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# find dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate message
###############################################
#   MSG_EULER_FILE
set(MSG_EULER_FILE
  msg/EulerFloat.msg
  msg/EulerInt16.msg
  msg/EventValue.msg
)
###############################################
#   BUILD_DEVELOP
if(BUILD_DEVELOP)
  set(MSG_DEVELOP_FILE
    msg/sample/MrsSample.msg
  )
else(BUILD_DEVELOP)
  set(MSG_DEVELOP_FILE "")
endif()
###############################################

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_DEVELOP_FILE}
  ${MSG_EULER_FILE}
  msg/BodyMessage.msg
  msg/CastResume.msg
  msg/ClusterArm.msg
  msg/ClusterLeg.msg
  msg/ClusterLocation.msg
  msg/DancingMessage.msg
  msg/FramesNo.msg
  msg/GazingAhead.msg
  msg/MotionDataEulerInt16.msg
  msg/Instructions.msg

  msg/Collected.msg
  msg/OrderReceptionist.msg
  msg/BellInfo.msg
  msg/HtmlPage.msg

  msg/BoundingBox.msg
  msg/BoundingBoxes.msg
  msg/StateBehavior.msg

  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
