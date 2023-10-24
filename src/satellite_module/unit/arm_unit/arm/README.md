# Arm controller

## インストール

### Micro ROS

```bash
source /opt/MaidRobotSystem/source/ros/env.sh

sudo apt install -y ros-cmake-modules
mkdir -p ${MRS_WORKSPACE}/.colcon/src && cd ${MRS_WORKSPACE}/.colcon
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep install --from-paths src --ignore-src -y
rosdep update
${MRS_WORKSPACE}/source/ros/build.sh ${MRS_WORKSPACE}/.colcon/src BUILD release micro_ros_setup
source ${MRS_WORKSPACE}/.colcon/install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source ${MRS_WORKSPACE}/.colcon/install/local_setup.bash
```

## 動作方法


```bash
source /opt/MaidRobotSystem/source/ros/env.sh

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baud 115200
```
