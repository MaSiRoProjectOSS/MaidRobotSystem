#!/bin/bash

## ================================
## Settings
WORK_FOLDER=`cd ${1:-/opt/MaidRobotSystem} && pwd`
WORK_FOLDER=`readlink -f ${WORK_FOLDER}`
SCRIPT_FOLDER=$(cd $(dirname $0) && pwd)
SCRIPT_FOLDER=`readlink -f ${SCRIPT_FOLDER}`

PARAM_01=$2
PARAM_02=$3
PARAM_03=$4
PARAM_04=$5
## ================================
## Escape sequence
COLOR_ON_GREEN="\e[32m"
COLOR_ON_RED="\e[31m"
COLOR_OFF="\e[m"
## ================================
source ${SCRIPT_FOLDER}/env.sh
## ================================

if [ ! -z "${WORK_FOLDER}" ]
then
    if [ -d "${WORK_FOLDER}" ]
    then
        cd ${WORK_FOLDER}

        export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/
        source /usr/share/colcon_cd/function/colcon_cd.sh
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

        echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
        echo -e "${COLOR_ON_GREEN}  ROS           :${COLOR_OFF} ${ROS_DISTRO}"
        echo -e "${COLOR_ON_GREEN}  ROS IP        :${COLOR_OFF} ${ROS_IP}"
        echo -e "${COLOR_ON_GREEN}  ROS Domain    :${COLOR_OFF} ${ROS_DOMAIN_ID}"
        echo -e "${COLOR_ON_GREEN}  CAST          :${COLOR_OFF} ${MRS_MY_NAME_IS} [ID:${MRS_CAST_ID}]"
        echo -e "${COLOR_ON_GREEN}  Target Fodler :${COLOR_OFF} ${MRS_WORKSPACE}"
        echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
        echo -e "${COLOR_ON_GREEN}[command]${COLOR_OFF} ros2 ${PARAM_01} ${PARAM_02} ${PARAM_03} ${PARAM_04}"
        echo -e ""
        ros2 ${PARAM_01} ${PARAM_02} ${PARAM_03} ${PARAM_04}
        echo -e "${COLOR_ON_GREEN}-------------------------------------------------------------------------------${COLOR_OFF}"
    else
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}  Could not find the folder : ${MRS_WORKSPACE}${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
    fi
else
    echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
    echo -e "${COLOR_ON_RED}  Specify a folder as the argument.${COLOR_OFF}"
    echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
fi
