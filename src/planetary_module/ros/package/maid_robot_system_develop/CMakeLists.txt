cmake_minimum_required(VERSION 3.8)
project(maid_robot_system_develop)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(maid_robot_system REQUIRED)

# generate message
#rosidl_generate_interfaces(${PROJECT_NAME}
#  DEPENDENCIES std_msgs geometry_msgs
# )

# /////////////////////////////////////////////
# add node
# /////////////////////////////////////////////

add_subdirectory(nodes/unit/mrs_cloud_node)
add_subdirectory(nodes/unit/template_node)

# /////////////////////////////////////////////

include_directories(include)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
