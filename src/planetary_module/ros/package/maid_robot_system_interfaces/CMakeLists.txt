cmake_minimum_required(VERSION 3.8)
project(maid_robot_system_interfaces)

# Select build unit
set(MRS_BUILD_DEVELOP ON)

if(DEFINED ENV{MRS_BUILD_DEVELOP})
  set(MRS_BUILD_DEVELOP $ENV{MRS_BUILD_DEVELOP})
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
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate message
###############################################
#   MRS_BUILD_DEVELOP
if(MRS_BUILD_DEVELOP)
  set(MSG_DEVELOP_FILE
    msg/develop/MrsSample.msg
  )
endif()
###############################################
#   MSG_MATH_FILE
set(MSG_MATH_FILE
  msg/math/EulerFloat.msg
  msg/math/EulerInt16.msg
  msg/math/Vector3.msg
  msg/math/Quaternion.msg
  msg/math/Rect.msg
  msg/math/RectFloat.msg
)
#   MSG_POSE_FILE
set(MSG_POSE_FILE
  msg/pose/PoseDetection.msg
  msg/pose/Landmark.msg
  msg/pose/BodySide.msg
  msg/pose/PoseLandmarkModel.msg
)
#   MSG_ARUCO_FILE
set(MSG_ARUCO_FILE
  msg/aruco/ArMarkers.msg
)
#   MSG_MRS_FILE
set(MSG_MRS_FILE
  msg/mrs/MrsEye.msg
  msg/mrs/MrsNeck.msg
  msg/mrs/MrsLip.msg
  msg/mrs/MrsVoice.msg
)

#   SRV_FILES
set(SRV_FILES
  srv/MediaPipePoseLandmarkDetection.srv
  srv/VideoCapture.srv
)
###############################################

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_DEVELOP_FILE}
  ${MSG_MATH_FILE}
  ${MSG_POSE_FILE}
  ${MSG_ARUCO_FILE}
  ${MSG_MRS_FILE}
  ${SRV_FILES}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

include_directories(include)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
