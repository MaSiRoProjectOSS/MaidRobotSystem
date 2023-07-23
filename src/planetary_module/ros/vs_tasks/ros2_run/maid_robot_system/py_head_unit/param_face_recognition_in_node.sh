
#!/bin/bash
############################################################################
echo "----------------"
echo "ros2 param get /maid_robot_system/miko/face_recognition_in_node ..."
echo "----------------"
echo "  image"
echo "    image/height              : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  image/height`
echo "    image/width               : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  image/width`
echo "    image/overlay_information : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  image/overlay_information`
echo "    image/publish             : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  image/publish`
echo "----------------"
echo "  confidence"
echo "    confidence/min_detection  : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  confidence/min_detection`
echo "    confidence/min_tracking   : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  confidence/min_tracking`
echo "    confidence/visibility_th  : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  confidence/visibility_th`

echo "----------------"
echo "  device/left"
echo "    device/left/height        : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  device/left/height`
echo "    device/left/width         : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  device/left/width`
echo "    device/left/by_path       : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  device/left/by_path`
echo "    device/left/id            : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  device/left/id`
echo "    device/left/type          : "`ros2 param get /maid_robot_system/miko/left_face_recognition_in_node  device/left/type`

echo "  device/right"
echo "    device/right/height       : "`ros2 param get /maid_robot_system/miko/right_face_recognition_in_node  device/right/height`
echo "    device/right/width        : "`ros2 param get /maid_robot_system/miko/right_face_recognition_in_node  device/right/width`
echo "    device/right/by_path      : "`ros2 param get /maid_robot_system/miko/right_face_recognition_in_node  device/right/by_path`
echo "    device/right/id           : "`ros2 param get /maid_robot_system/miko/right_face_recognition_in_node  device/right/id`
echo "    device/right/type         : "`ros2 param get /maid_robot_system/miko/right_face_recognition_in_node  device/right/type`
echo "----------------"
echo "  info"
echo "    info/verbose              : "`ros2 param get /maid_robot_system/miko/right_face_recognition_in_node  info/verbose`
echo "----------------"
echo "    update                    : "`ros2 param get /maid_robot_system/miko/right_face_recognition_in_node  update`
echo "----------------"

# ros2 param set /maid_robot_system/miko/face_recognition_in_node   info/verbose     True
# ros2 param set /maid_robot_system/miko/face_recognition_in_node   info/verbose     False
# ros2 param set /maid_robot_system/miko/face_recognition_in_node   update           True
