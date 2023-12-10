#!/bin/bash

echo "==================================="
echo "03_add_startup.sh"
echo "==================================="

cd `dirname $0`
sudo cp file/service/MaidRobotSystem_HeadUnit.service /lib/systemd/system/MaidRobotSystem_HeadUnit.service

sudo systemctl enable MaidRobotSystem_HeadUnit
sudo systemctl daemon-reload
sudo systemctl list-unit-files --type=service | grep MaidRobotSystem_HeadUnit
# sudo systemctl status     MaidRobotSystem_HeadUnit
# sudo systemctl enable     MaidRobotSystem_HeadUnit
# sudo systemctl disable    MaidRobotSystem_HeadUnit
# sudo systemctl start      MaidRobotSystem_HeadUnit
# sudo systemctl stop       MaidRobotSystem_HeadUnit
