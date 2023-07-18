#!/bin/bash
MY_NODE_NAME=$1
MY_PACKAGE_NAME=$2

echo source $(readlink -f $(dirname $0))/env.sh


if [ -z "${MY_NODE_NAME}" ]; then
    echo "usage: $0 <node_name> <package_name>"
    exit 1
fi
if [ -z "${MY_PACKAGE_NAME}" ]; then
    echo "usage: $0 <node_name> <package_name>"
    exit 1
fi

ros2 pkg create \
    --build-type ament_cmake \
    --dependencies rclcpp rclcpp_components maid_robot_system_interfaces \
    --license MIT \
    --maintainer-name ${HOSTNAME} \
    --description 'This package is ...' \
    --maintainer-email developer@masiro-project.com \
    --node-name ${MY_NODE_NAME} ${MY_PACKAGE_NAME}
