#!/bin/bash

echo "==================================="
echo "01_06_install_Software.sh"
echo "==================================="

## =======================================
## Settings
## =======================================
INSTALL_EXFAT=false

## =======================================
## Install
## =======================================
sudo apt update
sudo apt upgrade -y

## Install QT
sudo apt-get install -y build-essential libfontconfig1 mesa-common-dev libglu1-mesa-dev qt*5-dev qml-module-qtquick-controls qml-module-qtquick-controls2

## Software for development environment
sudo apt-get install -y graphviz doxygen clang-format llvm jq fzf

## To be able to use exFAT format
if "${INSTALL_ROS_UROS}"; then
    sudo apt-get install -y exfat-fuse exfat-utils
    sudo apt-get install -y gparted
fi

## Install JDK
#sudo apt install -y openjdk-19-jdk
sudo apt install -y openjdk-17-jdk
