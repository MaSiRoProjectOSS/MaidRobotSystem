#!/bin/bash

source ../env.sh

## =======================================
## Settings
## =======================================
ROS_DISTRO=humble
INSTALL_ROS_DESKTOP=false
INSTALL_ROS_DEVELOPMENT_TOOLS=false
INSTALL_ROS_DDS=true
INSTALL_ROS_UROS=false

## ================================
## Escape sequence
COLOR_ON_RED="\e[31m"
COLOR_ON_BLUE="\e[34m"
COLOR_ON_GREEN="\e[32m"
COLOR_ON_YELLOW="\e[33m"
COLOR_OFF="\e[m"
## ================================
echo -e "${COLOR_ON_BLUE}===================================${COLOR_OFF}"

## =======================================
## Adding Authority
## =======================================
sudo gpasswd -a `whoami` dialout
sudo usermod -aG MaSiRoProject $USER
ret=$?
if [ 0 -ne $ret ]; then
    sudo groupadd MaSiRoProject
    sudo usermod -aG MaSiRoProject $USER
    echo -e "${COLOR_ON_BLUE}* Join group : MaSiRoProject${COLOR_OFF}"
    groups $USER
fi

## =======================================
## install ROS2
## =======================================
echo -e "${COLOR_ON_BLUE}* Install ROS2 [${ROS_DISTRO}]${COLOR_OFF}"

if [ ! -e "${MRS_WORKSPACE}" ]; then
    echo -e "${COLOR_ON_RED}   ERROR : Not found ros_workspace : ${MRS_WORKSPACE} ${COLOR_OFF}"
    exit 1
else
    sudo mkdir -p ${MRS_WORKSPACE}/.colcon
    if [ ! -e "/etc/apt/sources.list.d/ros2.list" ]; then
        sudo apt update
        sudo apt install -y software-properties-common
        sudo add-apt-repository universe
        sudo apt install -y curl gnupg lsb-release

        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt install -y locales
        sudo locale-gen en_US en_US.UTF-8
        sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    fi
    ## =======================================
    ## set LANG
    ## =======================================
    export LANG=en_US.UTF-8
    ## =======================================
    ## Update apt
    ## =======================================
    sudo apt autoremove -y
    sudo apt update
    sudo apt upgrade -y

    sudo apt install -y build-essential python3-pip
    sudo apt install -y ros-${ROS_DISTRO}-ros-base
    echo -e "${COLOR_ON_BLUE}   * Install : ros-${ROS_DISTRO}-ros-base${COLOR_OFF}"
    if "${INSTALL_ROS_DESKTOP}"; then
        sudo apt install -y ros-${ROS_DISTRO}-desktop
        echo -e "${COLOR_ON_BLUE}   * Install : ros-${ROS_DISTRO}-desktop${COLOR_OFF}"
    fi
    if "${INSTALL_ROS_DEVELOPMENT_TOOLS}"; then
        sudo apt install -y ros-dev-tools
        echo -e "${COLOR_ON_BLUE}   * Install : ros-dev-tools${COLOR_OFF}"
    fi
    if "${INSTALL_ROS_DDS}"; then
        sudo apt install -q -y rti-connext-dds-6.0.1
        echo -e "${COLOR_ON_BLUE}   * Install : rti-connext-dds-6.0.1${COLOR_OFF}"
        source /opt/rti.com/rti_connext_dds-6.0.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash;
    fi
    sudo apt purge ros-${ROS_DISTRO}-examples-*
    sudo apt install -y python3-rosdep libpython3-dev python3-pip

    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd
    #sudo rosdep init
    #rosdep update
fi

if [ ! -d "/opt/ros/${ROS_DISTRO}" ];
then
    echo -e "${COLOR_ON_RED}ERROR : Not support Distributions : ${ROS_DISTRO}${COLOR_OFF}"
    exit 1
fi

## =======================================
## Installation of related software
## =======================================
echo -e "${COLOR_ON_BLUE}* Install of related software${COLOR_OFF}"

# ROS
sudo apt install -y ros-${ROS_DISTRO}-serial-driver
sudo apt install -y ros-${ROS_DISTRO}-rosbridge-server
sudo apt install -y python3-colcon-common-extensions

# Application
sudo apt install -y nlohmann-json3-dev

## =======================================
## Install micro ros
## =======================================
if "${INSTALL_ROS_UROS}"; then
    #######################
    # Setup
    #######################
    # ここでコンパイルするためのソフトをインストール
    sudo apt install -y ros-cmake-modules
    # インストールするフォルダに移動
    mkdir -p ${MRS_WORKSPACE}/.colcon/src && cd ${MRS_WORKSPACE}/.colcon
    # mico-rosをインストールするためのsetupをダウンロード
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    # 依存関係をインストール
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    # micro-ros のsetupをコンパイル
    ${MRS_WORKSPACE}/source/ros/build.sh ${MRS_WORKSPACE}/.colcon/src BUILD release micro_ros_setup
    # micro-ros のsetupを使えるようにする
    source ${MRS_WORKSPACE}/.colcon/install/local_setup.bash

    #######################
    # micor-ros を使えるようにする
    #######################
    # micor-ros Agentをダウンロード
    ros2 run micro_ros_setup create_agent_ws.sh
    # micor-ros Agentをダウンロード
    ros2 run micro_ros_setup build_agent.sh
    # micor-ros Agentを使えるようにする
    source ${MRS_WORKSPACE}/.colcon/install/local_setup.bash
    # 下記は使うときのコマンド
    # ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baud 115200
    # ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baud 115200
fi

# ###################################################################

echo -e "${COLOR_ON_GREEN}===================================${COLOR_OFF}"
echo -e "${COLOR_ON_GREEN} FINISHED INSTALLED${COLOR_OFF}"
echo -e "${COLOR_ON_GREEN}===================================${COLOR_OFF}"
