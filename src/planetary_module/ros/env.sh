#!/bin/bash
# ///////////////////////////////////////////////////////////////////
# source /opt/MaidRobotSystem/src/planetary_module/ros/script/env.sh
# ///////////////////////////////////////////////////////////////////

function mrs_help {
    ## =======================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_BLUE="\e[34m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    ## =======================================
    echo -e ${COLOR_ON_GREEN}"./mrs.sh "${COLOR_ON_BLUE}"[--mrs_config=FILE_NAME] ..."${COLOR_OFF}
    echo -e "  Enables calling functions for MaidRobotSystem."
    echo -e ${COLOR_ON_BLUE}"    --mrs_config=FILE_NAME  "${COLOR_OFF}": Set json file if you want to replace the config file."
    echo -e ""
    ## ---------------------------
    echo -e ${COLOR_ON_GREEN}"mrs_help"${COLOR_OFF}
    echo -e "  Displays a list of available commands and help."
    ## ---------------------------
    echo -e ${COLOR_ON_GREEN}"mrs_env_print "${COLOR_ON_BLUE}"MRS_CONFIG"${COLOR_OFF}
    echo -e "  Outputs the set environment variables."
    ## ---------------------------
    echo -e ${COLOR_ON_GREEN}"mrs_load_config "${COLOR_ON_BLUE}"[MRS_CONFIG]"${COLOR_OFF}
    echo -e "  The information in the json file set in MRS_CONFIG is used as the environment setting."
    ## ---------------------------
    echo -e ${COLOR_ON_GREEN}"mrs_save_config "${COLOR_ON_BLUE}"MRS_CONFIG"${COLOR_OFF}
    echo -e "  Register config.json."
    ## ---------------------------
    echo -e ${COLOR_ON_GREEN}"mrs_build "${COLOR_ON_BLUE}"WORK_COMMANDS WORK_BUILD_TYPE WORK_PACKAGES_SELECT WORK_ARG_OPTION"${COLOR_OFF}
    echo -e ${COLOR_ON_BLUE}"    WORK_COMMANDS        "${COLOR_OFF}": rebuild or build or clean"
    echo -e ${COLOR_ON_BLUE}"    WORK_BUILD_TYPE      "${COLOR_OFF}": debug or release"
    echo -e ${COLOR_ON_BLUE}"    WORK_PACKAGES_SELECT "${COLOR_OFF}": (your package name)"
    echo -e ${COLOR_ON_BLUE}"    WORK_ARG_OPTION      "${COLOR_OFF}": (add option)"
    echo -e ${COLOR_ON_GREEN}"mrs_create_node "${COLOR_ON_BLUE}"NODE_NAME PACKAGE_NAME ..."${COLOR_OFF}
    echo -e ${COLOR_ON_BLUE}"    NODE_NAME         "${COLOR_OFF}": name of the empty executable"
    echo -e ${COLOR_ON_BLUE}"    PACKAGE_NAME      "${COLOR_OFF}": The package name"
    echo -e "    Please check this command for other options."
    echo -e "      ros2 pkg create --help"

    echo -e ""
    ## ---------------------------
    echo -e ${COLOR_ON_GREEN}"aros2 ..."${COLOR_OFF}
    echo -e "  Override ros2 command."
    echo -e "    Please check this command for other options."
    echo -e "      ros2 --help"
    ## ---------------------------
    echo -e ${COLOR_ON_GREEN}"aros2_param_list "${COLOR_ON_BLUE}"[NODE_NAME]"${COLOR_OFF}
    echo -e ${COLOR_ON_BLUE}"    NODE_NAME    "${COLOR_OFF}": name of the empty executable"
    ## ---------------------------
    echo -e ${COLOR_ON_GREEN}"aros2_topic_pub "${COLOR_ON_BLUE}"TOPIC_NAME MESSAGE_TYPE YAML_FILE ..."${COLOR_OFF}
    echo -e ${COLOR_ON_BLUE}"    TOPIC_NAME   "${COLOR_OFF}": Name of the ROS topic to publish to (e.g. '/chatter')${COLOR_OFF}"
    echo -e ${COLOR_ON_BLUE}"    MESSAGE_TYPE "${COLOR_OFF}": Type of the ROS message (e.g. 'std_msgs/String')${COLOR_OFF}"
    echo -e ${COLOR_ON_BLUE}"    YAML_FILE    "${COLOR_OFF}": Values to fill the message with in YAML file${COLOR_OFF}"
    echo -e "    Please check this command for other options."
    echo -e "      ros2 topic pub --help"
}

## =======================================
## FUNCTION : COMMON
## =======================================
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

function mrs_env_print {
    ## ================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_BLUE="\e[34m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_OFF="\e[m"
    ## ================================
    export MRS_CONFIG=${1:-${MRS_CONFIG}}
    if [ ! -f "${MRS_CONFIG}" ]; then
        local CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
        if [ -f "${CURRENT_DIR}/config/${MRS_CONFIG}" ]; then
            export MRS_CONFIG=${CURRENT_DIR}/config/${MRS_CONFIG}
        elif [ -f "${CURRENT_DIR}/config/default.json" ]; then
            export MRS_CONFIG=${CURRENT_DIR}/config/default.json
        else
            export MRS_CONFIG=/opt/MaidRobotSystem/data/config.json
        fi
    fi
    if [ -f ${MRS_CONFIG} ]; then
        echo -e ${COLOR_ON_BLUE}"MRS_CONFIG: ${COLOR_ON_GREEN}${MRS_CONFIG}"${COLOR_OFF}
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
        local CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
        if [ -f "${CURRENT_DIR}/config/${MRS_CONFIG}" ]; then
            export MRS_CONFIG=${CURRENT_DIR}/config/${MRS_CONFIG}
        elif [ -f "${CURRENT_DIR}/config/default.json" ]; then
            export MRS_CONFIG=${CURRENT_DIR}/config/default.json
        else
            export MRS_CONFIG=/opt/MaidRobotSystem/data/config.json
        fi
    fi
    if [ -f ${MRS_CONFIG} ]; then
        local CONFIG_ARRAY=("CAST" "MRS" "ROS_VARIABLES" "BUILD")
        local JSON_DATA=$(cat ${MRS_CONFIG})
        local CONFIG_RAW=""
        for CONFIG_RAW in "${CONFIG_ARRAY[@]}"
        do
            local EXPORTLIST=$(echo ${JSON_DATA} | jq -r '.'${CONFIG_RAW}' | to_entries[] | [.key, .value] | @tsv')
            IFS=$'\n'
            mrs_set_export ${EXPORTLIST}
        done
    fi
    IFS=$' \t\n'
}

function mrs_save_config {
    local ret=0
    ## ================================
    ## Escape sequence
    local COLOR_ON_GREEN="\e[32m"
    local COLOR_ON_BLUE="\e[34m"
    local COLOR_ON_RED="\e[31m"
    local COLOR_ON=${COLOR_ON_GREEN}
    local COLOR_OFF="\e[m"
    ## ================================
    local ret=0
    export MRS_CONFIG=${1:-${MRS_CONFIG}}
    if [ ! -f "${MRS_CONFIG}" ]; then
        local CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
        if [ -f "${CURRENT_DIR}/config/${MRS_CONFIG}" ]; then
            export MRS_CONFIG=${CURRENT_DIR}/config/${MRS_CONFIG}
        elif [ -f "${CURRENT_DIR}/config/default.json" ]; then
            export MRS_CONFIG=${CURRENT_DIR}/config/default.json
        else
            export MRS_CONFIG=/opt/MaidRobotSystem/data/config.json
        fi
        ret=1
    fi
    if [ 0 -ne ${ret} ]; then
        COLOR_ON=${COLOR_ON_RED}
    fi
    echo -e "${COLOR_ON}===============================================================================${COLOR_OFF}"
    echo -e "${COLOR_ON}MRS CONFIG : ${MRS_CONFIG}${COLOR_OFF}"
    if [ 0 -ne ${ret} ]; then
    echo -e "${COLOR_ON}  ERROR MESSAGE: Default file specified.${COLOR_OFF}"
    fi
    echo -e "${COLOR_ON}===============================================================================${COLOR_OFF}"
    return "${ret}"
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
     if [ -z "${MRS_ROS_LOG_LEVEL}" ] ;then
        export MRS_ROS_LOG_LEVEL="INFO"
    else
        case "${MRS_ROS_LOG_LEVEL^^}" in
            F* )
                export MRS_ROS_LOG_LEVEL="FATAL"
                ;;
            E* )
                export MRS_ROS_LOG_LEVEL="ERROR"
                ;;
            W* )
                export MRS_ROS_LOG_LEVEL="WARN"
                ;;
            I* )
                export MRS_ROS_LOG_LEVEL="INFO"
                ;;
            D* )
                export MRS_ROS_LOG_LEVEL="DEBUG"
                ;;
            * )
                export MRS_ROS_LOG_LEVEL="INFO"
                ;;
        esac
    fi

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

## =======================================
## FUNCTION
## =======================================

function mrs_build {
    export BUILD_DATE=`date +%Y%m%d%H%M%S`
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

    mrs_load_config ${MRS_CONFIG}
    aros2_default

    if [ -n "${ROS_DISTRO}" ] ;then
        ## ================================
        ## command exsample:
        ## /opt/MaidRobotSystem/source/ros/build.sh <build | rebuild | clean> <debug | release> [packages-select]
        ## ================================
        local TARGET_FOLDER=${MRS_ROS_PACKAGE_FOLDER}
        local WORK_COMMANDS=${1:-"build"}
        local WORK_BUILD_TYPE=${2:-${BUILD_TYPE}}
        local WORK_PACKAGES_SELECT=${3:-""}
        shift
        shift
        shift
        local WORK_COLCON_PARAMETE=""
        local WORK_ARG_OPTION=""
        local WORK_CMAKE_ARGS=""
        while (( $# > 0 ))
        do
            WORK_ARG_OPTION=" "$1
            shift
        done
        COMPLILEDATE=`date +"%Y%m%d_%H%M%S"`
        COMPLILEDATE_TEXT=`date +"%Y/%m/%d/ %H:%M:%S"`
        ## ================================
        ##  "BUILD_LOG_LEVEL": "CRITICAL"/"ERROR"/"WARNING"/"INFO"/"DEBUG"/"NOTSET"
        if [ -z "${BUILD_LOG_LEVEL}" ] ;then
            COLCON_LOG_LEVEL="INFO"
        else
            case "${BUILD_LOG_LEVEL^^}" in
                "CRITICAL" )    COLCON_LOG_LEVEL="CRITICAL" ;;
                "ERROR" )       COLCON_LOG_LEVEL="ERROR" ;;
                "WARNING" )     COLCON_LOG_LEVEL="WARNING" ;;
                "INFO" )        COLCON_LOG_LEVEL="INFO" ;;
                "DEBUG" )       COLCON_LOG_LEVEL="DEBUG" ;;
                "NOTSET" )      COLCON_LOG_LEVEL="NOTSET" ;;
                * )             COLCON_LOG_LEVEL="INFO" ;;
            esac
        fi
        if [[ "debug" == "${WORK_BUILD_TYPE,,}" ]]; then
            WORK_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Debug"
        else
            WORK_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release"
        fi
        ## ================================

        local MRS_COLCON=${MRS_WORKSPACE}/.colcon
        mkdir -p ${MRS_COLCON}
        pushd ${MRS_COLCON} > /dev/null
            ## ================================
            if [ -n "${BUILD_CMAKE_ARGS}" ]; then
                WORK_CMAKE_ARGS=" "${BUILD_CMAKE_ARGS}
            fi
            if [ ! -z "${WORK_CMAKE_ARGS}" ]; then
                WORK_CMAKE_ARGS="--cmake-args "${WORK_CMAKE_ARGS}
            fi

            TARGET_FOLDER_ARG="--base-paths ${TARGET_FOLDER} --paths ${MRS_COLCON}"

            ## ================================

            if [ ! -z "${TARGET_FOLDER}" ]
            then
                if [ -d "${TARGET_FOLDER}" ]
                then
                    if [ "clean" = "${WORK_COMMANDS,,}" ]; then
                        echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                        echo -e "${COLOR_ON_BLUE}Command:${COLOR_OFF}"
                         echo -e "rm -rf ${MRS_COLCON}"
                        echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                        rm -rf ${MRS_COLCON}/*
                        history -s ${MRS_COLCON}/*
                    else
                        if [ "rebuild" = "${WORK_COMMANDS,,}" ]; then
                            WORK_COLCON_PARAMETE="--cmake-clean-first --cmake-clean-cache"
                        fi

                        source /usr/share/colcon_cd/function/colcon_cd.sh
                        export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/
                        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

                        MAKEFLAGS=-j4

                        echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
                        if [ ! -z "${WORK_PACKAGES_SELECT}" ]; then
                            WORK_CMAKE_ARGS=${WORK_CMAKE_ARGS}" --packages-select "${WORK_PACKAGES_SELECT}
                        fi
                        echo -e "${COLOR_ON_BLUE}Command:${COLOR_OFF}"
                        echo -e "colcon build ${BUILD_COLCON_OPTION} ${TARGET_FOLDER_ARG} ${WORK_CMAKE_ARGS} ${WORK_ARG_OPTION} ${WORK_COLCON_PARAMETE}"
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
        echo -e "${COLOR_ON}ROS                           :${COLOR_OFF} ${ROS_DISTRO}"
        echo -e "${COLOR_ON}Colcon Workspace              :${COLOR_OFF} ${MRS_COLCON}"
        echo -e "${COLOR_ON}Target Folder                 :${COLOR_OFF} ${TARGET_FOLDER}"
        echo -e "${COLOR_ON}Colcon${COLOR_OFF}"
        echo -e "${COLOR_ON}  Log level                   :${COLOR_OFF} ${COLCON_LOG_LEVEL}"
        echo -e "${COLOR_ON}ENV${COLOR_OFF}"
        echo -e "${COLOR_ON}  ROS${COLOR_OFF}"
        echo -e "${COLOR_ON}    ROS_DOMAIN_ID             :${COLOR_OFF} ${ROS_DOMAIN_ID}"
        echo -e "${COLOR_ON}    ROS_LOCALHOST_ONLY        :${COLOR_OFF} ${ROS_LOCALHOST_ONLY}"
        echo -e "${COLOR_ON}    ROS_IP                    :${COLOR_OFF} ${ROS_IP}"
        echo -e "${COLOR_ON}    ROS_MASTER_URI            :${COLOR_OFF} ${ROS_MASTER_URI}"
        echo -e "${COLOR_ON}  MRS${COLOR_OFF}"
        echo -e "${COLOR_ON}    MRS_CAST_ID               :${COLOR_OFF} ${MRS_CAST_ID}"
        echo -e "${COLOR_ON}    MRS_CAST_NAME             :${COLOR_OFF} ${MRS_CAST_NAME}"
        echo -e "${COLOR_ON}    MRS_CAST_DATA             :${COLOR_OFF} ${MRS_CAST_DATA}"
        echo -e "${COLOR_ON}    MRS_ROS_NAMESPACE         :${COLOR_OFF} ${MRS_ROS_NAMESPACE}"

        echo -e "${COLOR_ON}BUILD${COLOR_OFF}"
        echo -e "${COLOR_ON}    MRS_BUILD_DEVELOP         :${COLOR_OFF} ${MRS_BUILD_DEVELOP}"
        echo -e "${COLOR_ON}    MRS_BUILD_HEAD_UNIT       :${COLOR_OFF} ${MRS_BUILD_HEAD_UNIT}"
        echo -e "${COLOR_ON}    MRS_BUILD_ARM_UNIT        :${COLOR_OFF} ${MRS_BUILD_ARM_UNIT}"
        echo -e "${COLOR_ON}    MRS_BUILD_WAIST_DOWN_UNIT :${COLOR_OFF} ${MRS_BUILD_WAIST_DOWN_UNIT}"
        echo -e "${COLOR_ON}    MRS_BUILD_MOBILITY_UNIT   :${COLOR_OFF} ${MRS_BUILD_MOBILITY_UNIT}"
        echo -e "${COLOR_ON}    MRS_BUILD_CLOUD_UNIT      :${COLOR_OFF} ${MRS_BUILD_CLOUD_UNIT}"
        echo -e "${COLOR_ON}    MRS_BUILD_MANAGEMENT_UNIT :${COLOR_OFF} ${MRS_BUILD_MANAGEMENT_UNIT}"
        echo -e "${COLOR_ON}===============================================================================${COLOR_OFF}"
    fi

    if [ 0 -ne ${ret} ]; then
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}  Build failed(${ret}).${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}===============================================================================${COLOR_OFF}"
    fi
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
    #ARGS_IN+=("--build-type ament_python")
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
            echo -e "${COLOR_ON}  COMMAND   :${COLOR_OFF} ros2 pkg create --description 'This package is ...' ${ARGS_IN[@]}"
            echo -e "${COLOR_ON}===============================================================================${COLOR_OFF}"
        popd > /dev/null
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
    local NODE_NAME=$1

    ## ================================
    mrs_load_config ${MRS_CONFIG}
    aros2_default
    ## ================================

    if [ -z "${NODE_NAME}" ]; then
        NODE_NAME=$(ros2 node list | grep 'maid_' | fzf)
    fi
    if [ "--help" = "${NODE_NAME}" ]; then
        echo -e "${COLOR_ON_RED}usage: aros2_param_list NODE_NAME${COLOR_OFF}"
        echo -e ""
        echo -e "${COLOR_ON_RED}  positional arguments::${COLOR_OFF}"
        echo -e "${COLOR_ON_RED}    NODE_NAME  : The node name${COLOR_OFF}"
        return
    fi
    if [ ! -z "${NODE_NAME}" ]; then
        local PARAM_ARRAY=($(ros2 param list ${NODE_NAME}))
        local PARAM_COUNT=${#PARAM_ARRAY[@]}
        local RAW_COUNT=0
        echo "----------------"
        echo "ros2 param get ${NODE_NAME} ..."
        for PARAM_RAW in "${PARAM_ARRAY[@]}"
        do
            RAW_COUNT=$((RAW_COUNT+1))
            value=$(ros2 param get ${NODE_NAME} ${PARAM_RAW})
            printf "  [%03s/%03s] %-35s : %s\n" "${RAW_COUNT}" "${PARAM_COUNT}" "${PARAM_RAW:0:34}" "${value}"
        done
        history -s aros2_param_list ${NODE_NAME}
    else
        echo -e "${COLOR_ON_RED}Not found node${COLOR_OFF}"
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
        local CMD_OPTIONS=()
        while (( $# > 0 ))
        do
            CMD_OPTIONS+=($1)
            shift
        done
        ## ================================
        mrs_load_config ${MRS_CONFIG}
        aros2_default
        ## ================================

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
