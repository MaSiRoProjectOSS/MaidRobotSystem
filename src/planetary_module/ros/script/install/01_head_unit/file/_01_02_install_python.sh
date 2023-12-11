#!/bin/bash

echo "==================================="
echo "01_02_install_python.sh"
echo "==================================="

## update apt
sudo apt update
sudo apt upgrade -y

## install python3-pip
sudo apt install -y python3-pip
sudo python3 -m pip install --upgrade pip
# sudo python3 -m pip install -r file/requirements.txt

## Jetson health monitoring
#sudo apt install -y python-pip
#sudo -H pip install jetson-stats

sudo pip install mediapipe
