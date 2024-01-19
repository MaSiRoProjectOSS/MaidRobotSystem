#!/bin/bash

echo "==================================="
echo "01_02_install_python.sh"
echo "==================================="
CURRENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

## update apt
sudo apt update
sudo apt upgrade -y

## install python3-pip
sudo apt install -y python3-pip
sudo apt install -y python3-venv
sudo python3 -m pip install --upgrade pip
sudo python3 -m pip install -r $CURRENT_DIR/requirements.txt

sudo apt install -y python3-cv-bridge



## Jetson health monitoring
#sudo -H pip install jetson-stats

