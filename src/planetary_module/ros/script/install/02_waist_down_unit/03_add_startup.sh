#!/bin/bash

echo "==================================="
echo "03_add_startup.sh"
echo "==================================="

cd `dirname $0`
sudo cp file/service/MaidRobotSystem_WaistDownUnit.service /lib/systemd/system/MaidRobotSystem_WaistDownUnit.service

sudo systemctl enable MaidRobotSystem_WaistDownUnit
sudo systemctl daemon-reload
sudo systemctl list-unit-files --type=service | grep MaidRobotSystem_WaistDownUnit
# sudo systemctl status     MaidRobotSystem_WaistDownUnit
# sudo systemctl enable     MaidRobotSystem_WaistDownUnit
# sudo systemctl disable    MaidRobotSystem_WaistDownUnit
# sudo systemctl start      MaidRobotSystem_WaistDownUnit
# sudo systemctl stop       MaidRobotSystem_WaistDownUnit
