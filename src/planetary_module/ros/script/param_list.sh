
#!/bin/bash
############################################################################

if [ $# -lt 1 ]; then
    echo "Usage: $0 <package_name>"
    exit 1
fi

PACKAGE_NAME=$1

PARAM_ARRAY=($(ros2 param list ${PACKAGE_NAME}))
PARAM_COUNT=${#PARAM_ARRAY[@]}
RAW_COUNT=0
echo "----------------"
echo "ros2 param get ${PACKAGE_NAME} ..."
for PARAM_RAW in "${PARAM_ARRAY[@]}"
do
    RAW_COUNT=$((RAW_COUNT+1))
    value=$(ros2 param get ${PACKAGE_NAME} ${PARAM_RAW})
    printf "  [%03s/%03s] %-35s : %s\n" "${RAW_COUNT}" "${PARAM_COUNT}" "${PARAM_RAW:0:34}" "${value}"
done

# ros2 param set /maid_robot_system/miko/recognition_in_node   info/verbose     True
# ros2 param set /maid_robot_system/miko/recognition_in_node   info/verbose     False
# ros2 param set /maid_robot_system/miko/recognition_in_node   update           True
