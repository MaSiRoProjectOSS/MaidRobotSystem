#!/bin/bash

VOS_IP_ADDR=${1:-`hostname -I | awk '{print $1}'`}

CURRENT_DIR=$(cd $(dirname $0) && pwd)
source ${CURRENT_DIR}/src/vosk.config.ini

echo ------------------------------------------
echo Start Vosk : ws://${VOS_IP_ADDR}:${SERVER_PORT}
docker start vosk_server_${VOSK_MODEL}
echo ------------------------------------------
python3 ${CURRENT_DIR}/src/client.py -u ws://${VOS_IP_ADDR}:${SERVER_PORT}
