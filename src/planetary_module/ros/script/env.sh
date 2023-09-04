#!/bin/bash
# ///////////////////////////////////////////////////////////////////
# source /opt/MaidRobotSystem/src/planetary_module/ros/script/env.sh
# ///////////////////////////////////////////////////////////////////

## =======================================
## Escape sequence
COLOR_ON_RED="\e[31m"
COLOR_OFF="\e[m"
## =======================================

## =======================================
## Settings : Default
## =======================================
### Settings : SYSTEM
export LANG=en_US.UTF-8
export BUILD_DATE=`date +%Y%m%d%H%M%S`
if [ -z "${ARRAY_ROS_DISTRO}" ]; then
    ARRAY_ROS_DISTRO=("iron" "humble" "rolling")
fi
### Settings : Maid robot system
export MRS_WORKSPACE=${MRS_WORKSPACE:-/opt/MaidRobotSystem}
export MRS_CAST_NAME=${MRS_CAST_NAME:-`hostname`}
export MRS_CAST_ID=${MRS_CAST_ID:-1000}

## =======================================
## Settings : Default : ROS
## =======================================
### ROS 1
if [ -z "${ROS_IP}" ]; then
    export ROS_IP=`hostname -I  | cut -d " " -f 1`
fi
if [ -z "${ROS_MASTER_URI}" ]; then
    export ROS_MASTER_URI=http://${ROS_IP}:11311
fi

### ROS 2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

## =======================================
## Settings : Load config file
## =======================================
CONFIG_PATH=${1:-/opt/MaidRobotSystem/data/config.json}
if [ ! -f ${CONFIG_PATH} ]; then
    WORK_DIR=$(cd $(dirname $0) && pwd)
    CONFIG_PATH=`readlink -f ${WORK_DIR}/config.json `
fi
if [ -f ${CONFIG_PATH} ]; then
    CONFIG_ARRAY=("CAST" "MRS" "ROS_VARIABLES" "BUILD")
    JSON_DATA=$(cat ${CONFIG_PATH})
    for CONFIG_RAW in "${CONFIG_ARRAY[@]}"
    do
        export JSON_KEY=$(echo ${JSON_DATA} | jq -c '.'$CONFIG_RAW)
        if [ "null" != "${JSON_KEY}" ]; then
            echo ${JSON_KEY} | jq -r ' to_entries[] | [.key, .value] | @tsv' | {
                while read key value; do
                    buf=$(eval echo ${value})
                    export $key="${buf}"
                done
            }
        fi
    done
fi

## =======================================
## Check ROS Distributions
## =======================================
if [ ! -z "${ROS_DISTRO}" ]; then
    if [ ! -d "/opt/ros/${ROS_DISTRO}" ]; then
        ROS_DISTRO=""
    fi
fi
if [ -z "${ROS_DISTRO}" ]; then
    for DISTRO_RAW in "${ARRAY_ROS_DISTRO[@]}"
    do
        if [ -d "/opt/ros/${DISTRO_RAW}" ]; then
            ROS_DISTRO=${DISTRO_RAW}
            break
        fi
    done
fi
## ###########################################################################
## =======================================
## Settings
## =======================================

if [ -z "${ROS_DISTRO}" ]; then
    echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
    echo -e "${COLOR_ON_RED}  Not support ROS Distributions. Please install ROS 2.${COLOR_OFF}"
    echo -e "${COLOR_ON_RED}========================================${COLOR_OFF}"
else
    if [ ! -z "${ROS_LOG_DIR}" ]; then
        if [ ! -d "${ROS_LOG_DIR}" ]; then
            mkdir -p ${ROS_LOG_DIR}
        fi
    fi
    ## =======================================
    ## Configuring environment
    ## =======================================
    if [ -d "/opt/ros/${ROS_DISTRO}" ]; then
        export AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
        export CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
        source /opt/ros/${ROS_DISTRO}/setup.bash
        source /opt/ros/${ROS_DISTRO}/local_setup.bash
    fi

    WORK_RTI_CONNEXT_DDS=$(readlink -f /opt/rti.com/rti_connext_dds-*/resource/scripts/rtisetenv_x64Linux*.bash)
    if [ -f "${WORK_RTI_CONNEXT_DDS}" ]; then
        source ${WORK_RTI_CONNEXT_DDS}; > /dev/null 2>&1
    fi

    if [ -f "${MRS_WORKSPACE}/.colcon/install/local_setup.bash" ]; then
        source ${MRS_WORKSPACE}/.colcon/install/local_setup.bash
    fi
fi
