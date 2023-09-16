#!/bin/bash
# ///////////////////////////////////////////////////////////////////
# source /opt/MaidRobotSystem/src/planetary_module/ros/script/env.sh
# ///////////////////////////////////////////////////////////////////

## =======================================
## FUNCTION
## =======================================
function mrs_help {
    ## =======================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    ## =======================================
    echo -e "mrs_help        : Display command list"
    echo -e "mrs_build       : Run MRS build"
    echo -e "mrs_create_node NODE_NAME PACKAGE_NAME : Creating a ROS package template"
    echo -e "    NODE_NAME     : Name of the empty executable"
    echo -e "    PACKAGE_NAME  : The package nam$"
    echo -e "mrs_load_config : "
    #echo -e "mrs_set_export  : "
    #echo -e "mrs_default     : "

    echo -e "aros2           : "
    echo -e "aros2_topic_pub : "
    echo -e "aros2_param_list: "
    #echo -e "aros_default   : "
}

function mrs_build {
    BUILD_DATE=`date +%Y%m%d%H%M%S`
    ############################################################################
    ##
    ## mrs_build WORK_COMMANDS WORK_BUILD_TYPE WORK_PACKAGES_SELECT WORK_ARG_OPTION
    ##
    ##  WORK_COMMANDS        : rebuild or build or clean
    ##  WORK_BUILD_TYPE      : debug or release
    ##  WORK_PACKAGES_SELECT : (your package name)
    ##  WORK_ARG_OPTION      : (add option)
    ##
    ##  exsample:
    ##      ./build.sh build release
    ##
    local COLOR_ON_RED="\e[31m"
    local COLOR_ON_BLUE="\e[34m"
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_YELLOW="\e[33m"
    local COLOR_ON=${COLOR_ON_BLUE}
    local COLOR_OFF="\e[m"
    ## ================================
    ret=0
    ADD_LOG_LEVEL=false

    mrs_load_config ${MRS_CONFIG}
    aros2_default
    ## ================================
    ##  "BUILD_LOG_LEVEL": "CRITICAL"/"ERROR"/"WARNING"/"INFO"/"DEBUG"/"NOTSET"
    case "$BUILD_LOG_LEVEL" in
        "CRITICAL" )    ADD_LOG_LEVEL=true  ;;
        "ERROR" )       ADD_LOG_LEVEL=true  ;;
        "WARNING" )     ADD_LOG_LEVEL=true  ;;
        "INFO" )        ADD_LOG_LEVEL=true  ;;
        "DEBUG" )       ADD_LOG_LEVEL=true  ;;
        "NOTSET" )      ADD_LOG_LEVEL=true  ;;
    esac

    if [ -n "${ROS_DISTRO}" ] ;then
        ## ================================
        ## command exsample:
        ## /opt/MaidRobotSystem/source/ros/build.sh <build | rebuild | clean> <debug | release> [packages-select]
        ## ================================
        TARGET_FOLDER=${MRS_ROS_PACKAGE_FOLDER}
        WORK_COMMANDS=${1:-"build"}
        WORK_BUILD_TYPE=${2:-"release"}
        WORK_PACKAGES_SELECT=${3:-""}
        shift
        shift
        shift
        local WORK_ARG_OPTION=""
        while (( $# > 0 ))
        do
            WORK_ARG_OPTION=" "$1
            shift
        done
        COMPLILEDATE=`date +"%Y%m%d_%H%M%S"`
        COMPLILEDATE_TEXT=`date +"%Y/%m/%d/ %H:%M:%S"`
        ## ================================
        ## Escape sequence
        COLOR_ON_RED="\e[31m"
        COLOR_OFF="\e[m"
        local MRS_COLCON=${MRS_WORKSPACE}/.colcon
        mkdir -p ${MRS_COLCON}
        pushd ${MRS_COLCON} > /dev/null
            ## ================================
            if [ -n "${CMAKE_ARGS}" ]; then
                WORK_CMAKE_ARGS="--cmake-args "${CMAKE_ARGS}
            else
                WORK_CMAKE_ARGS=""
            fi

            TARGET_FOLDER_ARG="--base-paths ${TARGET_FOLDER} --paths ${MRS_COLCON}"

            ## ================================
            WORK_COLCON_PARAMETE=""

            if [ ! -z "${TARGET_FOLDER}" ]
            then
                if [ -d "${TARGET_FOLDER}" ]
                then
                    if [ "clean" = "${WORK_COMMANDS,,}" ]; then
                        rm -rf ${MRS_COLCON}/*
                        history -s ${MRS_COLCON}/*
                        echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                        echo -e "${COLOR_ON_BLUE}Command:${COLOR_OFF} rm -rf ${MRS_COLCON}"
                        echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                    else
                        if [ "rebuild" = "${WORK_COMMANDS,,}" ]; then
                            #echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                            #echo "rebuild (delete install folder)"
                            #rm -rf ${TARGET_FOLDER}/install
                            #rm -rf ${TARGET_FOLDER}/log
                            #rm -rf ${TARGET_FOLDER}/build
                            WORK_COLCON_PARAMETE="--cmake-clean-first --cmake-clean-cache"
                        fi
                        if [ true = ${ADD_LOG_LEVEL} ]; then
                            COLCON_LOG_LEVEL=${BUILD_LOG_LEVEL^^}
                        fi

                        source /usr/share/colcon_cd/function/colcon_cd.sh
                        export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/
                        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
                        if [ -e ${TARGET_FOLDER}/install/ ]; then
                            source ${TARGET_FOLDER}/install/local_setup.bash
                        fi
                        if [ -e ${MRS_COLCON}/install/ ]; then
                            source ${MRS_COLCON}/install/local_setup.bash
                        fi

                        MAKEFLAGS=-j4

                        echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                        if [ ! -z "${WORK_PACKAGES_SELECT}" ]; then
                            WORK_CMAKE_ARGS=${WORK_CMAKE_ARGS}" --packages-select "${WORK_PACKAGES_SELECT}" --allow-overriding "${WORK_PACKAGES_SELECT}" --allow-overriding maid_robot_system_interfaces"
                        else
                            WORK_CMAKE_ARGS=${WORK_CMAKE_ARGS}" --allow-overriding maid_robot_system_interfaces"
                        fi
                        echo -e "${COLOR_ON_BLUE}Command:${COLOR_OFF} colcon build ${BUILD_COLCON_OPTION} ${TARGET_FOLDER_ARG} ${WORK_CMAKE_ARGS} ${WORK_ARG_OPTION} ${WORK_COLCON_PARAMETE}"
                        echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                        colcon build ${BUILD_COLCON_OPTION} ${TARGET_FOLDER_ARG} ${WORK_CMAKE_ARGS} ${WORK_ARG_OPTION} ${WORK_COLCON_PARAMETE}
                        ret=$?
                        history -s colcon build ${BUILD_COLCON_OPTION} ${TARGET_FOLDER_ARG} ${WORK_CMAKE_ARGS} ${WORK_ARG_OPTION} ${WORK_COLCON_PARAMETE}
                        if [ 0 -eq ${ret} ]; then
                            if [ -e ${MRS_COLCON}/install/ ]; then
                                source ${MRS_COLCON}/install/local_setup.bash
                            fi
                        else
                            COLOR_ON=${COLOR_ON_RED}
                        fi
                    fi
                else
                    echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
                    echo -e "${COLOR_ON_RED}  Could not find the folder :${COLOR_OFF} ${TARGET_FOLDER}"
                    echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
                fi
            else
                echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
                echo -e "${COLOR_ON_RED}  Specify a folder as the argument.${COLOR_OFF}"
                echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
            fi
        popd > /dev/null

    echo -e "${COLOR_ON}===============================================================================${COLOR_OFF}"
    echo -e "${COLOR_ON}Date :${COLOR_OFF} ${COMPLILEDATE_TEXT}"
    echo -e "${COLOR_ON}ROS                        :${COLOR_OFF} ${ROS_DISTRO}"
    echo -e "${COLOR_ON}Colcon Workspace           :${COLOR_OFF} ${MRS_COLCON}"
    echo -e "${COLOR_ON}Target Folder              :${COLOR_OFF} ${TARGET_FOLDER}"
    if [ true = ${ADD_LOG_LEVEL} ]; then
        echo -e "${COLOR_ON}Log level                  :${COLOR_OFF ${COLCON_LOG_LEVEL}}"
    fi
        echo -e "${COLOR_ON}ENV${COLOR_OFF}"
        echo -e "${COLOR_ON}  ROS${COLOR_OFF}"
        echo -e "${COLOR_ON}    ROS_DOMAIN_ID          :${COLOR_OFF} ${ROS_DOMAIN_ID}"
        echo -e "${COLOR_ON}    ROS_LOCALHOST_ONLY     :${COLOR_OFF} ${ROS_LOCALHOST_ONLY}"
        echo -e "${COLOR_ON}    ROS_IP                 :${COLOR_OFF} ${ROS_IP}"
        echo -e "${COLOR_ON}    ROS_MASTER_URI         :${COLOR_OFF} ${ROS_MASTER_URI}"
        echo -e "${COLOR_ON}  MRS${COLOR_OFF}"
        echo -e "${COLOR_ON}    MRS_CAST_ID            :${COLOR_OFF} ${MRS_CAST_ID}"
        echo -e "${COLOR_ON}    MRS_SKIN_HOME          :${COLOR_OFF} ${MRS_CAST_NAME}"
        echo -e "${COLOR_ON}    MRS_ROS_NAMESPACE      :${COLOR_OFF} ${MRS_ROS_NAMESPACE}"
        echo -e "${COLOR_ON}    MRS_SKIN_HOME          :${COLOR_OFF} ${MRS_SKIN_HOME}"

        echo -e "${COLOR_ON}BUILD${COLOR_OFF}"
        echo -e "${COLOR_ON}    BUILD_DEVELOP          :${COLOR_OFF} ${BUILD_DEVELOP}"
        echo -e "${COLOR_ON}    BUILD_HEAD_UNIT        :${COLOR_OFF} ${BUILD_HEAD_UNIT}"
        echo -e "${COLOR_ON}    BUILD_ARM_UNIT         :${COLOR_OFF} ${BUILD_ARM_UNIT}"
        echo -e "${COLOR_ON}    BUILD_WAIST_DOWN_UNIT  :${COLOR_OFF} ${BUILD_WAIST_DOWN_UNIT}"
        echo -e "${COLOR_ON}    BUILD_MOBILITY_UNIT    :${COLOR_OFF} ${BUILD_MOBILITY_UNIT}"
        echo -e "${COLOR_ON}    BUILD_CLOUD_UNIT       :${COLOR_OFF} ${BUILD_CLOUD_UNIT}"
        echo -e "${COLOR_ON}    BUILD_MANAGEMENT_UNIT  :${COLOR_OFF} ${BUILD_MANAGEMENT_UNIT}"
        echo -e "${COLOR_ON}===============================================================================${COLOR_OFF}"
    fi

    if [ 0 -ne ${ret} ]; then
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}  Build failed(${ret}).${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
    fi
}


function mrs_set_export {
    IFS=$'\t'
    while (( $# > 0 ))
    do
        ARR=(${1// / })
        buf=$(eval echo ${ARR[1]})
        export ${ARR[0]}=${buf}
        shift
    done
    IFS=$' \t\n'
}

function mrs_set_echo {
    ## ================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_BLUE="\e[34m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    ## ================================
    IFS=$'\t'
    while (( $# > 0 ))
    do
        ARR=(${1// / })
        buf=$(eval echo ${ARR[1]})
        echo -e "  ${COLOR_ON_BLUE}${ARR[0]}: ${COLOR_ON_GREEN}${buf}${COLOR_OFF}"
        shift
    done
    IFS=$' \t\n'
}

function mrs_env {
    ## ================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_BLUE="\e[34m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    ## ================================
    export MRS_CONFIG=${1:-${MRS_CONFIG}}
    if [ ! -f "${MRS_CONFIG}" ]; then
        export MRS_CONFIG=/opt/MaidRobotSystem/data/config.json
    fi
    if [ -f ${MRS_CONFIG} ]; then
        local CONFIG_ARRAY=("CAST" "MRS" "ROS_VARIABLES" "BUILD")
        local JSON_DATA=$(cat ${MRS_CONFIG})
        local CONFIG_RAW=""
        for CONFIG_RAW in "${CONFIG_ARRAY[@]}"
        do
            echo -e "${COLOR_ON_BLUE}${CONFIG_RAW}:${COLOR_OFF}"
            local JSON_KEY=$(echo ${JSON_DATA} | jq -c '.'$CONFIG_RAW)
            if [ "null" != "${JSON_KEY}" ]; then
                local EXPORTLIST=$(echo ${JSON_KEY} | jq -r ' to_entries[] | [.key, .value] | @tsv')
                IFS=$'\n'
                mrs_set_echo ${EXPORTLIST}
            fi
        done
    fi
    IFS=$' \t\n'
}

function mrs_load_config {
    export MRS_CONFIG=${1:-${MRS_CONFIG}}
    if [ ! -f "${MRS_CONFIG}" ]; then
        export MRS_CONFIG=/opt/MaidRobotSystem/data/config.json
    fi
    if [ -f ${MRS_CONFIG} ]; then
        local CONFIG_ARRAY=("CAST" "MRS" "ROS_VARIABLES" "BUILD")
        local JSON_DATA=$(cat ${MRS_CONFIG})
        local CONFIG_RAW=""
        for CONFIG_RAW in "${CONFIG_ARRAY[@]}"
        do
            local JSON_KEY=$(echo ${JSON_DATA} | jq -c '.'$CONFIG_RAW)
            if [ "null" != "${JSON_KEY}" ]; then
                local EXPORTLIST=$(echo ${JSON_KEY} | jq -r ' to_entries[] | [.key, .value] | @tsv')
                IFS=$'\n'
                mrs_set_export ${EXPORTLIST}
            fi
        done
    fi
    IFS=$' \t\n'
}

function mrs_save_config {
    export MRS_CONFIG=${1:-/opt/MaidRobotSystem/data/config.json}
    if [ -z "${MRS_CONFIG}" ]; then
        export MRS_CONFIG=/opt/MaidRobotSystem/data/config.json
    fi
    if [ ! -f "${MRS_CONFIG}" ]; then
        export MRS_CONFIG=/opt/MaidRobotSystem/data/config.json
    fi
    # echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
    # echo -e "${COLOR_ON_GREEN}MRS CONFIG : ${MRS_CONFIG}.${COLOR_OFF}"
    # echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
}

function mrs_create_node {
    ## ================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    local COLOR_ON=${COLOR_ON_GREEN}
    ## ================================
    local MY_NODE_NAME=$1
    local MY_PACKAGE_NAME=$2
    shift
    shift

    ## ================================
    mrs_load_config ${MRS_CONFIG}
    aros2_default
    ## ================================
    local DESCRIPTION_TEXT="This packageis..."
    local ARGS_IN=()
    ARGS_IN+=("--destination-directory ${MRS_ROS_PACKAGE_FOLDER}")
    ARGS_IN+=("--build-type ament_cmake")
    #ARGS_IN+=("--description '${DESCRIPTION_TEXT}'")
    ARGS_IN+=("--license MIT")
    ARGS_IN+=("--dependencies rclcpp rclcpp_components maid_robot_system_interfaces")
    ARGS_IN+=("--maintainer-name $(git config --global user.name)")
    ARGS_IN+=("--maintainer-email developer@masiro-project.com")
    ARGS_IN+=("--node-name ${MY_NODE_NAME}")
    while (( $# > 0 ))
    do
        ARGS_IN+=($1)
        shift
    done
    ARGS_IN+=(${MY_PACKAGE_NAME})

    if [ -z "${MY_NODE_NAME}" ]; then
        echo -e "${COLOR_ON_RED}usage: mrs_create_node NODE_NAME PACKAGE_NAME${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}    NODE_NAME     : Name of the empty executable${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}    PACKAGE_NAME  : The package nam${COLOR_OFF}"
    elif [ -z "${MY_PACKAGE_NAME}" ]; then
        echo -e "${COLOR_ON_RED}usage: mrs_create_node NODE_NAME PACKAGE_NAME${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}    NODE_NAME     : Name of the empty executable${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}    PACKAGE_NAME  : The package nam${COLOR_OFF}"
    else
        pushd ${MRS_ROS_PACKAGE_FOLDER} > /dev/null
            ros2 pkg create --description 'This package is ...' ${ARGS_IN[@]}
            ret=$?
            history -s mrs_create_node ${MY_NODE_NAME} ${MY_PACKAGE_NAME}

            if [ 0 -ne ${ret} ]; then
                COLOR_ON=${COLOR_ON_RED}
            fi
            echo -e "${COLOR_ON}===============================================================================${COLOR_OFF}"
            if [ 0 -ne ${ret} ]; then
                echo -e "${COLOR_ON}[ERROR]${COLOR_OFF}"
            fi
            echo -e "${COLOR_ON}  DIRECTORY :${COLOR_OFF} ${MRS_ROS_PACKAGE_FOLDER}/${MY_PACKAGE_NAME}"
            echo -e "${COLOR_ON}  NODE NAME :${COLOR_OFF} ${MY_NODE_NAME}"
            echo -e "${COLOR_ON}  COMMAND   : ros2 pkg create --description 'This package is ...' ${ARGS_IN[@]}${COLOR_OFF}"
            echo -e "${COLOR_ON}===============================================================================${COLOR_OFF}"
        popd > /dev/null
    fi
}

function mrs_default {
    ### Settings : SYSTEM
    export LANG=en_US.UTF-8
    ### Settings : Maid robot system
    export MRS_WORKSPACE=${MRS_WORKSPACE:-/opt/MaidRobotSystem}
    export MRS_CAST_NAME=${MRS_CAST_NAME:-`hostname`}
    export MRS_CAST_ID=${MRS_CAST_ID:-1000}
}

function aros2_default {
    ## ================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    ## ================================
    if [ -z "${ARRAY_ROS_DISTRO}" ]; then
        ARRAY_ROS_DISTRO=("iron" "humble" "rolling")
    fi
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
    ## Settings
    ## =======================================
    if [ ! -z "${ROS_DISTRO}" ]; then
        if [ ! -d "/opt/ros/${ROS_DISTRO}" ]; then
            export ROS_DISTRO=""
        fi
    fi
    if [ -z "${ROS_DISTRO}" ]; then
        for DISTRO_RAW in "${ARRAY_ROS_DISTRO[@]}"
        do
            if [ -d "/opt/ros/${DISTRO_RAW}" ]; then
                export ROS_DISTRO=${DISTRO_RAW}
                break
            fi
        done
    fi

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

        WORK_RTI_CONNEXT_DDS=$(readlink -f /opt/rti.com/rti_connext_dds-*/resource/scripts/rtisetenv*.bash)
        if [ -f "${WORK_RTI_CONNEXT_DDS}" ]; then
            source ${WORK_RTI_CONNEXT_DDS}; > /dev/null 2>&1
        fi

        if [ -f "${MRS_WORKSPACE}/.colcon/install/local_setup.bash" ]; then
            source ${MRS_WORKSPACE}/.colcon/install/local_setup.bash
        fi
    fi
}


function aros2 {
    ## ================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    ## ================================
    ## Settings

    IFS=$' \t\n'
    local ARGS_IN=()
    while (( $# > 0 ))
    do
        ARGS_IN+=($1)
        shift
    done

    mrs_load_config ${MRS_CONFIG}
    aros2_default

    if [ ! -z "${MRS_WORKSPACE}" ]
    then
        if [ -d "${MRS_WORKSPACE}" ]
        then
            pushd ${MRS_WORKSPACE} > /dev/null
                export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/
                source /usr/share/colcon_cd/function/colcon_cd.sh
                source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

                echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
                echo -e "${COLOR_ON_GREEN}  ROS                 :${COLOR_OFF} ${ROS_DISTRO}"
                echo -e "${COLOR_ON_GREEN}  ROS IP              :${COLOR_OFF} ${ROS_IP}"
                echo -e "${COLOR_ON_GREEN}  ROS DOMAIN ID       :${COLOR_OFF} ${ROS_DOMAIN_ID}"
                echo -e "${COLOR_ON_GREEN}  ROS LOCALHOST ONLY  :${COLOR_OFF} ${ROS_LOCALHOST_ONLY}"
                echo -e "${COLOR_ON_GREEN}  CAST                :${COLOR_OFF} ${MRS_CAST_NAME} [ID:${MRS_CAST_ID}]"
                echo -e "${COLOR_ON_GREEN}  Target Fodler       :${COLOR_OFF} ${MRS_WORKSPACE}"
                echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
                echo -e "${COLOR_ON_GREEN}[command]${COLOR_OFF} ros2 ${ARGS_IN[@]}"
                echo -e ""
                ros2 ${ARGS_IN[@]}
                history -s aros2 ${ARGS_IN[@]}
                history -s ros2 ${ARGS_IN[@]}
                echo -e "${COLOR_ON_GREEN}-------------------------------------------------------------------------------${COLOR_OFF}"
            popd > /dev/null
        else
            echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
            echo -e "${COLOR_ON_RED}  Could not find the folder :${COLOR_OFF} ${MRS_WORKSPACE}"
            echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
        fi
    else
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}  Specify a folder as the argument.${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
    fi
}

function aros2_param_list {
    ## ================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    local COLOR_ON=${COLOR_ON_GREEN}
    ## ================================
    local PACKAGE_NAME=$1

    ## ================================
    mrs_load_config ${MRS_CONFIG}
    aros2_default
    ## ================================

    if [ -z "${PACKAGE_NAME}" ]; then
        PACKAGE_NAME=$(ros2 pkg list | grep 'maid_' | fzf)
    fi
    if [ ! -z "${PACKAGE_NAME}" ]; then
        local PARAM_ARRAY=($(ros2 param list ${PACKAGE_NAME}))
        local PARAM_COUNT=${#PARAM_ARRAY[@]}
        local RAW_COUNT=0
        echo "----------------"
        echo "ros2 param get ${PACKAGE_NAME} ..."
        for PARAM_RAW in "${PARAM_ARRAY[@]}"
        do
            RAW_COUNT=$((RAW_COUNT+1))
            value=$(ros2 param get ${PACKAGE_NAME} ${PARAM_RAW})
            printf "  [%03s/%03s] %-35s : %s\n" "${RAW_COUNT}" "${PARAM_COUNT}" "${PARAM_RAW:0:34}" "${value}"
        done
        history -s aros2_param_list ${PACKAGE_NAME}
    else
        echo -e "${COLOR_ON_RED}Not found package${COLOR_OFF}"
    fi
}

function aros2_topic_pub {
    ## ================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    ## ================================
    ## Settings
    if [ $# -lt 3 ]; then
        echo -e "${COLOR_ON_RED}aros2_topic_pub topic_name message_type yaml_file [OPTIONS]${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}    topic_name   : Name of the ROS topic to publish to (e.g. '/chatter')${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}    message_type : Type of the ROS message (e.g. 'std_msgs/String')${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}    yaml_file    : Values to fill the message with in YAML file${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}    [OPTIONS]"
        ros2 topic pub --help
        echo -e "${COLOR_OFF}"
    else
        local CMD_TOPIC_NAME=$1
        local CMD_MESSAGE_TYPE=$2
        local CMD_YAML_FILE=`cat $3`
        shift
        shift
        shift
        echo ========================================
        local CMD_OPTIONS=()
        while (( $# > 0 ))
        do
            CMD_OPTIONS+=($1)
            shift
        done
        echo ========================================
        ## ================================
        mrs_load_config ${MRS_CONFIG}
        aros2_default
        ## ================================
        echo ========================================

        if [ ! -z "${MRS_WORKSPACE}" ]
        then
            if [ -d "${MRS_WORKSPACE}" ]
            then
        echo ========================================
                pushd ${MRS_WORKSPACE}
        echo ========================================
                    export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/
                    source /usr/share/colcon_cd/function/colcon_cd.sh
                    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

                    echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
                    echo -e "${COLOR_ON_GREEN}  ROS                 :${COLOR_OFF} ${ROS_DISTRO}"
                    echo -e "${COLOR_ON_GREEN}  ROS IP              :${COLOR_OFF} ${ROS_IP}"
                    echo -e "${COLOR_ON_GREEN}  ROS Domain          :${COLOR_OFF} ${ROS_DOMAIN_ID}"
                    echo -e "${COLOR_ON_GREEN}  ROS LOCALHOST ONLY  :${COLOR_OFF} ${ROS_LOCALHOST_ONLY}"
                    echo -e "${COLOR_ON_GREEN}  CAST                :${COLOR_OFF} ${MRS_CAST_NAME} [ID:${MRS_CAST_ID}]"
                    echo -e "${COLOR_ON_GREEN}  Target Fodler       :${COLOR_OFF} ${MRS_WORKSPACE}"
                    echo -e "${COLOR_ON_GREEN}===============================================================================${COLOR_OFF}"
                    echo -e "${COLOR_ON_GREEN}[command]${COLOR_OFF} ros2 topic pub ${CMD_OPTIONS[@]} ${CMD_TOPIC_NAME} ${CMD_MESSAGE_TYPE} \"${CMD_YAML_FILE}\""
                    echo -e ""
                    ros2 topic pub ${CMD_OPTIONS[@]} ${CMD_TOPIC_NAME} ${CMD_MESSAGE_TYPE} "${CMD_YAML_FILE}"
                    history -s topic pub ${CMD_OPTIONS[@]} ${CMD_TOPIC_NAME} ${CMD_MESSAGE_TYPE} "${CMD_YAML_FILE}"
                    echo -e "${COLOR_ON_GREEN}-------------------------------------------------------------------------------${COLOR_OFF}"
                popd > /dev/null
            else
                echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
                echo -e "${COLOR_ON_RED}  Could not find the folder :${COLOR_OFF} ${MRS_WORKSPACE}"
                echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
            fi
        else
            echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
            echo -e "${COLOR_ON_RED}  Specify a folder as the argument.${COLOR_OFF}"
            echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
        fi
    fi
}

## =======================================
## Settings
## =======================================
mrs_load_config ${1:-${MRS_CONFIG}}
mrs_default
aros2_default
