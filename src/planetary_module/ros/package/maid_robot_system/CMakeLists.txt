cmake_minimum_required(VERSION 3.8)
project(maid_robot_system)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-deprecated-declarations
    -Wno-unused-parameter
    -Wno-unused-function
    -Wno-implicit-fallthrough
    -Wno-unused-variable
  )
endif()
#add_compile_options(--warn-unused-cli)

# generate file
write_file(${CMAKE_CURRENT_SOURCE_DIR}/include/maid_robot_system/config/__generate_config.hpp
  "#ifndef MAID_ROBOT_SYSTEM_CONFIG_GENERATE_CONFIG_HPP\n"
  "#define MAID_ROBOT_SYSTEM_CONFIG_GENERATE_CONFIG_HPP\n"
  "\n"
  "#define CONFIG_PATH_HOME \"${CMAKE_CURRENT_SOURCE_DIR}\"\n"
  "#define CONFIG_SKIN_NAME \"$ENV{MRS_MY_NAME_IS}\"\n"
  "#define CONFIG_CAST_ID    $ENV{MRS_CAST_ID}\n"
  "\n"
  "#endif\n"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate message
set(MSG_EULER_FILE
  msg/EulerFloat.msg
  msg/EulerInt16.msg
  msg/EventValue.msg
)
rosidl_generate_interfaces(${PROJECT_NAME}
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

# /////////////////////////////////////////////
# add node
# /////////////////////////////////////////////

add_subdirectory(nodes/condition_monitoring_node)
#add_subdirectory(nodes/hitomi_node)


# /////////////////////////////////////////////

include_directories(include)
install(DIRECTORY include/
  DESTINATION include
)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
