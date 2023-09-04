cmake_minimum_required(VERSION 3.8)
project(maid_robot_system)

# Select build unit
set(BUILD_HEAD_UNIT ON)
set(BUILD_ARM_UNIT ON)
set(BUILD_WAIST_DOWN_UNIT ON)
set(BUILD_MOBILITY_UNIT ON)
set(BUILD_CLOUD_UNIT ON)
set(BUILD_MANAGEMENT_UNIT ON)
set(BUILD_DEVELOP ON)

if(DEFINED ENV{BUILD_HEAD_UNIT})
  set(BUILD_HEAD_UNIT $ENV{BUILD_HEAD_UNIT})
endif()
if(DEFINED ENV{BUILD_ARM_UNIT})
  set(BUILD_ARM_UNIT $ENV{BUILD_ARM_UNIT})
endif()
if(DEFINED ENV{BUILD_WAIST_DOWN_UNIT})
  set(BUILD_WAIST_DOWN_UNIT $ENV{BUILD_WAIST_DOWN_UNIT})
endif()
if(DEFINED ENV{BUILD_MOBILITY_UNIT})
  set(BUILD_MOBILITY_UNIT $ENV{BUILD_MOBILITY_UNIT})
endif()
if(DEFINED ENV{BUILD_CLOUD_UNIT})
  set(BUILD_CLOUD_UNIT $ENV{BUILD_CLOUD_UNIT})
endif()
if(DEFINED ENV{BUILD_MANAGEMENT_UNIT})
  set(BUILD_MANAGEMENT_UNIT $ENV{BUILD_MANAGEMENT_UNIT})
endif()
if(DEFINED ENV{BUILD_DEVELOP})
  set(BUILD_DEVELOP $ENV{BUILD_DEVELOP})
endif()

# Select build unit
message(STATUS "Generate information")
message(STATUS "  MRS_SKIN_HOME  : $ENV{MRS_SKIN_HOME}")
message(STATUS "  MRS_CAST_NAME  : $ENV{MRS_CAST_NAME}")
message(STATUS "  MRS_CAST_ID    : $ENV{MRS_CAST_ID}")
message(STATUS "  BUILD_DATE     : $ENV{BUILD_DATE}")
message(STATUS "Build target")
message(STATUS "  BUILD_HEAD_UNIT       : $ENV{BUILD_HEAD_UNIT}")
message(STATUS "  BUILD_ARM_UNIT        : $ENV{BUILD_ARM_UNIT}")
message(STATUS "  BUILD_WAIST_DOWN_UNIT : $ENV{BUILD_WAIST_DOWN_UNIT}")
message(STATUS "  BUILD_MOBILITY_UNIT   : $ENV{BUILD_MOBILITY_UNIT}")
message(STATUS "  BUILD_CLOUD_UNIT      : $ENV{BUILD_CLOUD_UNIT}")
message(STATUS "  BUILD_MANAGEMENT_UNIT : $ENV{BUILD_MANAGEMENT_UNIT}")

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
  "#define CONFIG_PATH_HOME   \"$ENV{MRS_SKIN_HOME}\"\n"
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

if(BUILD_HEAD_UNIT)
  add_subdirectory(nodes/head_unit/mitumeru_node)
  add_subdirectory(nodes/head_unit/hitomi_node)
  add_subdirectory(nodes/head_unit/kubi_node)
  add_subdirectory(nodes/head_unit/voice_recognition_node)
endif()
if(BUILD_ARM_UNIT)
  message(STATUS "BUILD_ARM_UNIT : ON")
endif()
if(BUILD_WAIST_DOWN_UNIT)
  message(STATUS "BUILD_WAIST_DOWN_UNIT : ON")
endif()
if(BUILD_MOBILITY_UNIT)
  message(STATUS "BUILD_MOBILITY_UNIT : ON")
endif()
if(BUILD_CLOUD_UNIT)
  message(STATUS "BUILD_CLOUD_UNIT : ON")
endif()
if(BUILD_MANAGEMENT_UNIT)
  add_subdirectory(nodes/management_unit/condition_monitoring_node)
endif()
if(BUILD_DEVELOP)
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
