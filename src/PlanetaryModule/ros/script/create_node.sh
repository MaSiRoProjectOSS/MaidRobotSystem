#!/bin/bash
MY_NODE_NAME=$1
MY_PACKAGE_NAME=$2

source $(cd $(dirname $0)/../../source/ros/ && pwd)/env.sh


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
    --dependencies rclcpp rclcpp_components std_msgs mrs_interfaces \
    --license MIT \
    --node-name ${MY_NODE_NAME} ${MY_PACKAGE_NAME}
