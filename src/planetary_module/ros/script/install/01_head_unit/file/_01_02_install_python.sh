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
sudo python3 -m pip install -r requirements.txt

## Jetson health monitoring
#sudo -H pip install jetson-stats

