#!/bin/bash
############################################################################
##
## ./build.sh [WORK_FOLDER] [WORK_COMMANDS] [WORK_BUILD_TYPE] [WORK_PACKAGES_SELECT] [WORK_ARG_OPTION]
##
##  WORK_FOLDER          : ${MRS_WORKSPACE}
##  WORK_COMMANDS        : REBUILD or BUILD
##  WORK_BUILD_TYPE      : debug or release
##  WORK_PACKAGES_SELECT : (your package name)
##  WORK_ARG_OPTION      : (add option)
##
##  exsample:
##      ./build.sh ${MRS_WORKSPACE} BUILD release
##

COLOR_ON_RED="\e[31m"
COLOR_ON_BLUE="\e[34m"
COLOR_ON_GREEN="\e[32m"
COLOR_ON_YELLOW="\e[33m"
COLOR_OFF="\e[m"
## ================================
SCRIPT_FOLDER=$(cd $(dirname $0) && pwd)
#SCRIPT_FOLDER=`readlink -f ${SCRIPT_FOLDER}`
source ${SCRIPT_FOLDER}/env.sh

if [ -n "${ROS_DISTRO}" ] ;then
    ## ================================
    ## command exsample:
    ## /opt/MaidRobotSystem/source/ros/build.sh <workfolder> <BUILD | REBUILD> <debug | release> [packages-select]
    ## ================================
    ## Settings
    WORK_COLCON_OPTION="--symlink-install --parallel-workers 4"
    ## ================================
    WORK_FOLDER=`cd ${1:-${MRS_WORKSPACE}} && pwd`
    WORK_COMMANDS=${2:-""}
    WORK_BUILD_TYPE=${3:-""}
    WORK_PACKAGES_SELECT=${4:-""}
    WORK_ARG_OPTION=${5:-""}
    COMPLILEDATE=`date +"%Y%m%d_%H%M%S"`
    COMPLILEDATE_TEXT=`date +"%Y/%m/%d/ %H:%M:%S"`
    ## ================================
    ## Escape sequence
    COLOR_ON_RED="\e[31m"
    COLOR_OFF="\e[m"
    mkdir -p ${MRS_WORKSPACE}/.colcon
    cd ${MRS_WORKSPACE}/.colcon
    ## ================================

    WORK_CMAKE_ARGS=""
    WORK_FOLDER_ARG="--base-paths ${WORK_FOLDER} --paths ${MRS_WORKSPACE}/.colcon"

    ## ================================
    WORK_COLCON_PARAMETE=""

    if [ ! -z "${WORK_FOLDER}" ]
    then
        if [ -d "${WORK_FOLDER}" ]
        then
            if [ "clean" = "${WORK_COMMANDS}" ]; then
                rm -rf ${MRS_WORKSPACE}/.colcon/*
                echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
                echo -e "${COLOR_ON_GREEN}    rm -rf ${MRS_WORKSPACE}/.colcon${COLOR_OFF}"
                echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
            else
                if [ "REBUILD" = "${WORK_COMMANDS}" ]; then
                    #echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                    #echo "REBUILD (delete install folder)"
                    #rm -rf ${WORK_FOLDER}/install
                    #rm -rf ${WORK_FOLDER}/log
                    #rm -rf ${WORK_FOLDER}/build
                    WORK_COLCON_PARAMETE="--cmake-clean-first --cmake-clean-cache"
                fi
                if [ "debug" = "${WORK_BUILD_TYPE}" ]; then
                    WORK_CMAKE_ARGS=${WORK_CMAKE_ARGS}" -DCMAKE_BUILD_TYPE=Debug"
                    COLCON_LOG_LEVEL=DEBUG
                fi

                source /usr/share/colcon_cd/function/colcon_cd.sh
                export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/
                source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
                if [ -e ${WORK_FOLDER}/install/ ]; then
                    source ${WORK_FOLDER}/install/local_setup.bash
                fi
                if [ -e ${MRS_WORKSPACE}/.colcon/install/ ]; then
                    source ${MRS_WORKSPACE}/.colcon/install/local_setup.bash
                fi

                MAKEFLAGS=-j4

                echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                if [ ! -z "${WORK_PACKAGES_SELECT}" ]; then
                    WORK_CMAKE_ARGS=${WORK_CMAKE_ARGS}" --packages-select "${WORK_PACKAGES_SELECT}" --allow-overriding "${WORK_PACKAGES_SELECT}
                fi
                echo -e "${COLOR_ON_BLUE}Command: colcon build ${WORK_COLCON_OPTION} ${WORK_FOLDER_ARG} ${WORK_CMAKE_ARGS} ${WORK_ARG_OPTION} ${WORK_COLCON_PARAMETE}"
                echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                colcon build ${WORK_COLCON_OPTION} ${WORK_FOLDER_ARG} ${WORK_CMAKE_ARGS} ${WORK_ARG_OPTION} ${WORK_COLCON_PARAMETE}
                ret=$?
                if [ 0 -eq ${ret} ]; then
                    if [ -e ${MRS_WORKSPACE}/.colcon/install/ ]; then
                        source ${MRS_WORKSPACE}/.colcon/install/local_setup.bash
                    fi
                else
                    echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
                    echo -e "${COLOR_ON_RED}  Build failed(${ret}).${COLOR_OFF}"
                    echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
                fi
            fi
        else
            echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
            echo -e "${COLOR_ON_RED}  Could not find the folder : ${WORK_FOLDER}${COLOR_OFF}"
            echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
        fi
    else
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}  Specify a folder as the argument.${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
    fi

    echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}Date : ${COMPLILEDATE_TEXT}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}ROS                        : ${ROS_DISTRO}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}Workspace                  : ${MRS_WORKSPACE}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}Target folder              : ${WORK_FOLDER}${COLOR_OFF}"

    echo -e "${COLOR_ON_BLUE}ENV${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}  ROS${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    ROS_DOMAIN_ID          : ${ROS_DOMAIN_ID}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    ROS_IP                 : ${ROS_IP}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    ROS_MASTER_URI         : ${ROS_MASTER_URI}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}  MRS${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    MRS_CAST_ID            : ${MRS_CAST_ID}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    MRS_MY_NAME_IS         : ${MRS_MY_NAME_IS}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    MRS_SKIN_HOME          : ${MRS_SKIN_HOME}${COLOR_OFF}"

    echo -e "${COLOR_ON_BLUE}BUILD${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    BUILD_DEVELOP          : ${BUILD_DEVELOP}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    BUILD_HEAD_UNIT        : ${BUILD_HEAD_UNIT}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    BUILD_ARM_UNIT         : ${BUILD_ARM_UNIT}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    BUILD_WAIST_DOWN_UNIT  : ${BUILD_WAIST_DOWN_UNIT}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    BUILD_MOBILITY_UNIT    : ${BUILD_MOBILITY_UNIT}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    BUILD_CLOUD_UNIT       : ${BUILD_CLOUD_UNIT}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}    BUILD_MANAGEMENT_UNIT  : ${BUILD_MANAGEMENT_UNIT}${COLOR_OFF}"
    echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
fi
