#!/bin/bash

DIR_NAME=`dirname $0`

echo "==================================="
echo "03_add_startup.sh"
echo "==================================="
SERVICE_FILE_NAME=MaidRobotSystem_HeadUnit.service

echo "Settings:"
echo "  MRS_CONFIG=${MRS_CONFIG}"
echo "  SERVICE_FILE_NAME=${SERVICE_FILE_NAME}"
echo "-----------------------------------"

cd ${DIR_NAME}
sudo cp file/service/${SERVICE_FILE_NAME} /lib/systemd/system/${SERVICE_FILE_NAME}
sudo sed -i -e "s|__MRS_CONFIG__|${MRS_CONFIG}|g" /lib/systemd/system/${SERVICE_FILE_NAME}

sudo systemctl enable ${SERVICE_FILE_NAME}  > /dev/null 2>&1
sudo systemctl daemon-reload                > /dev/null 2>&1
sudo systemctl start ${SERVICE_FILE_NAME}   > /dev/null 2>&1
sudo systemctl list-unit-files --type=service show MaidRobotSystem*
# sudo systemctl status     MaidRobotSystem_HeadUnit
# sudo systemctl enable     MaidRobotSystem_HeadUnit
# sudo systemctl disable    MaidRobotSystem_HeadUnit
# sudo systemctl start      MaidRobotSystem_HeadUnit
# sudo systemctl stop       MaidRobotSystem_HeadUnit
# sudo systemctl restart    MaidRobotSystem_HeadUnit
