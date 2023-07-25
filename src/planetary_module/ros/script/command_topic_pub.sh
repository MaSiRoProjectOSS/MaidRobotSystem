#!/bin/bash

## ================================
## Settings
SCRIPT_FOLDER=$(cd $(dirname $0) && pwd)
SCRIPT_FOLDER=`readlink -f ${SCRIPT_FOLDER}`

CMD_TOPIC_NAME=$1
CMD_MESSAGE_TYPE=$2
CMD_YAML_FILE=`cat $3
CMD_OPTIONS=$4
## ================================
## Escape sequence
COLOR_ON_GREEN="\e[32m"
COLOR_ON_RED="\e[31m"
COLOR_OFF="\e[m"
## ================================
source ${SCRIPT_FOLDER}/env.sh
## ================================

if [ ! -z "${MRS_WORKSPACE}" ]
then
    if [ -d "${MRS_WORKSPACE}" ]
    then
        cd ${MRS_WORKSPACE}

        export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/
        source /usr/share/colcon_cd/function/colcon_cd.sh
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

        echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
        echo -e "${COLOR_ON_GREEN}  ROS                 :${COLOR_OFF} ${ROS_DISTRO}"
        echo -e "${COLOR_ON_GREEN}  ROS IP              :${COLOR_OFF} ${ROS_IP}"
        echo -e "${COLOR_ON_GREEN}  ROS Domain          :${COLOR_OFF} ${ROS_DOMAIN_ID}"
        echo -e "${COLOR_ON_GREEN}  ROS LOCALHOST ONLY  :${ROS_LOCALHOST_ONLY}${COLOR_OFF}"
        echo -e "${COLOR_ON_GREEN}  CAST                :${COLOR_OFF} ${MRS_CAST_NAME} [ID:${MRS_CAST_ID}]"
        echo -e "${COLOR_ON_GREEN}  Target Fodler       :${COLOR_OFF} ${MRS_WORKSPACE}"
        echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
        echo -e "${COLOR_ON_GREEN}[command]${COLOR_OFF} ros2 topic pub ${CMD_OPTIONS} ${CMD_TOPIC_NAME} ${CMD_MESSAGE_TYPE} \"${CMD_YAML_FILE}\""
        echo -e ""
        ros2 topic pub ${CMD_OPTIONS} ${CMD_TOPIC_NAME} ${CMD_MESSAGE_TYPE} "${CMD_YAML_FILE}"
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
