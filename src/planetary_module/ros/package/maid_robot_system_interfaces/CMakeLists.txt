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
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0 -pg")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -s -DNDEBUG -march=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g3 -Og -pg")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -s -DNDEBUG -march=native")
#add_compile_options(--warn-unused-cli)


# find dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate message
###############################################
#   MSG_EULER_FILE
set(MSG_EULER_FILE
  msg/euler/EulerFloat.msg
  msg/euler/EulerInt16.msg
)
###############################################
#   BUILD_DEVELOP
if(BUILD_DEVELOP)
  set(MSG_DEVELOP_FILE
    msg/sample/MrsSample.msg
  )
endif()
###############################################

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_DEVELOP_FILE}
  ${MSG_EULER_FILE}

  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(include)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
