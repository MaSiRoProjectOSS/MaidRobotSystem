#!/bin/bash
# ///////////////////////////////////////////////////////////////////

## =======================================
## Settings
## =======================================
ROS_DISTRO=humble
export MRS_WORKSPACE=`cd ${1:-/opt/MaidRobotSystem} && pwd`

## =======================================
## Configuring environment
## =======================================
if [ -d "/opt/ros/${ROS_DISTRO}" ]; then
    export AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
    export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source /opt/ros/${ROS_DISTRO}/local_setup.bash
else
    unset ROS_DISTRO
fi

if [ -d "/opt/rti.com/rti_connext_dds-6.0.1/resource/scripts" ]; then
    source /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash; > /dev/null 2>&1
fi

if [ -d "${MRS_WORKSPACE}/.colcon/install" ]; then
    source /${MRS_WORKSPACE}/.colcon/install/local_setup.bash
fi

## =======================================
## Settings : ROS
## =======================================
# mkdir -p ${MRS_WORKSPACE}/.ros/log
#export ROS_LOG_DIR=${MRS_WORKSPACE}/.ros_log
export ROS_HOME=${MRS_WORKSPACE}
export ROS_LOG_DIR=~/.ros
export RCUTILS_LOGGING_BUFFERED_STREAM=1
export RCUTILS_COLORIZED_OUTPUT=1
#export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
#export RCUTILS_LOGGING_USE_STDOUT=1

#if [ -e /opt/ros/${ROS_DISTRO} ]; then
#    export MASIRO_PROJECT_ROS_ROBOT_TYPE=robot_state_publisher
#else
#    export MASIRO_PROJECT_ROS_ROBOT_TYPE=state_publisher
#fi

## =======================================
## Settings : SYSTEM
## =======================================
export LANG=en_US.UTF-8

## =======================================
## Settings : SYSTEM
## =======================================
# Maid robot system
export MRS_MY_NAME_IS=`hostname`
export MRS_CAST_ID=1

# ROS 1
if [ -z "${ROS_IP}" ]; then
    export ROS_IP=`hostname -I  | cut -d " " -f 1`
fi
export ROS_MASTER_URI=http://${ROS_IP}:11311
# ROS 2
export ROS_DOMAIN_ID=0

# ###################################################################
