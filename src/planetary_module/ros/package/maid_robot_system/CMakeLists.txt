cmake_minimum_required(VERSION 3.8)
project(maid_robot_system)

set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0 -pg")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -s -DNDEBUG -march=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g3 -Og -pg")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -s -DNDEBUG -march=native")

# Select build unit
set(MRS_BUILD_HEAD_UNIT ON)
set(MRS_BUILD_ARM_UNIT ON)
set(MRS_BUILD_WAIST_DOWN_UNIT ON)
set(MRS_BUILD_MOBILITY_UNIT ON)
set(MRS_BUILD_CLOUD_UNIT ON)
set(MRS_BUILD_MANAGEMENT_UNIT ON)
set(MRS_BUILD_DEVELOP ON)

if(DEFINED ENV{MRS_BUILD_HEAD_UNIT})
  set(MRS_BUILD_HEAD_UNIT $ENV{MRS_BUILD_HEAD_UNIT})
endif()
if(DEFINED ENV{MRS_BUILD_ARM_UNIT})
  set(MRS_BUILD_ARM_UNIT $ENV{MRS_BUILD_ARM_UNIT})
endif()
if(DEFINED ENV{MRS_BUILD_WAIST_DOWN_UNIT})
  set(MRS_BUILD_WAIST_DOWN_UNIT $ENV{MRS_BUILD_WAIST_DOWN_UNIT})
endif()
if(DEFINED ENV{MRS_BUILD_MOBILITY_UNIT})
  set(MRS_BUILD_MOBILITY_UNIT $ENV{MRS_BUILD_MOBILITY_UNIT})
endif()
if(DEFINED ENV{MRS_BUILD_CLOUD_UNIT})
  set(MRS_BUILD_CLOUD_UNIT $ENV{MRS_BUILD_CLOUD_UNIT})
endif()
if(DEFINED ENV{MRS_BUILD_MANAGEMENT_UNIT})
  set(MRS_BUILD_MANAGEMENT_UNIT $ENV{MRS_BUILD_MANAGEMENT_UNIT})
endif()
if(DEFINED ENV{MRS_BUILD_DEVELOP})
  set(MRS_BUILD_DEVELOP $ENV{MRS_BUILD_DEVELOP})
endif()

# Select build unit
message(STATUS "Generate information")
message(STATUS "  MRS_CAST_NAME  : $ENV{MRS_CAST_NAME}")
message(STATUS "  MRS_CAST_ID    : $ENV{MRS_CAST_ID}")
message(STATUS "  MRS_CAST_DATA  : $ENV{MRS_CAST_DATA}")
message(STATUS "  BUILD_DATE     : $ENV{BUILD_DATE}")
message(STATUS "Build target")
message(STATUS "  MRS_BUILD_HEAD_UNIT       : $ENV{MRS_BUILD_HEAD_UNIT}")
message(STATUS "  MRS_BUILD_ARM_UNIT        : $ENV{MRS_BUILD_ARM_UNIT}")
message(STATUS "  MRS_BUILD_WAIST_DOWN_UNIT : $ENV{MRS_BUILD_WAIST_DOWN_UNIT}")
message(STATUS "  MRS_BUILD_MOBILITY_UNIT   : $ENV{MRS_BUILD_MOBILITY_UNIT}")
message(STATUS "  MRS_BUILD_CLOUD_UNIT      : $ENV{MRS_BUILD_CLOUD_UNIT}")
message(STATUS "  MRS_BUILD_MANAGEMENT_UNIT : $ENV{MRS_BUILD_MANAGEMENT_UNIT}")

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
  "#define CONFIG_FILE        \"$ENV{MRS_CAST_DATA}\"\n"
  "#define CONFIG_CAST_NAME   \"$ENV{MRS_CAST_NAME}\"\n"
  "#define CONFIG_CAST_ID     $ENV{MRS_CAST_ID}\n"
  "#define CONFIG_BUILD_DATE  \"$ENV{BUILD_DATE}\"\n"
  "\n"
  "#endif\n"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(maid_robot_system_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# /////////////////////////////////////////////
# add node
# /////////////////////////////////////////////

if(MRS_BUILD_HEAD_UNIT)
  add_subdirectory(nodes/head_unit/head_control_node)
  add_subdirectory(nodes/head_unit/eye_node)
  add_subdirectory(nodes/head_unit/neck_node)
  add_subdirectory(nodes/head_unit/lip_node)
  add_subdirectory(nodes/head_unit/voice_recognition_node)
  add_subdirectory(nodes/waist_down_unit/waist_down_control_node)
endif()
if(MRS_BUILD_ARM_UNIT)
  message(STATUS "MRS_BUILD_ARM_UNIT : ON")
endif()
if(MRS_BUILD_WAIST_DOWN_UNIT)
  message(STATUS "MRS_BUILD_WAIST_DOWN_UNIT : ON")
endif()
if(MRS_BUILD_MOBILITY_UNIT)
  message(STATUS "MRS_BUILD_MOBILITY_UNIT : ON")
endif()
if(MRS_BUILD_CLOUD_UNIT)
  message(STATUS "MRS_BUILD_CLOUD_UNIT : ON")
endif()
if(MRS_BUILD_MANAGEMENT_UNIT)
  add_subdirectory(nodes/management_unit/condition_monitoring_node)
endif()
if(MRS_BUILD_DEVELOP)
  add_subdirectory(nodes/develop/sample_node)
endif()

# /////////////////////////////////////////////

include_directories(include)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
