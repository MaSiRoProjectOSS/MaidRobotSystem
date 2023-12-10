#!/bin/bash

CURRENT_DIR=$(cd $(dirname $0) && pwd)
source ${CURRENT_DIR}/src/vosk.config.ini

sudo apt update
sudo apt -y upgrade
##------------------------------------------
docker --version
ret=$?

if [ 0 -eq ${ret} ]; then
    ##------------------------------------------
    sudo apt install -y python3-pip
    sudo apt install -y python3-pyaudio
    python3 -m pip install --upgrade pip
    pip install --upgrade websockets
    pip install --upgrade sounddevice
    echo "------------------------------------------"

    ##------------------------------------------
    DOCKER_VOSK_IMAGE_ID=`docker images vosk:${VOSK_MODEL} -q`
    if [ -z "${DOCKER_VOSK_IMAGE_ID}" ]; then
        echo "docker build vosk:${VOSK_MODEL}"
        docker build -t vosk:${VOSK_MODEL} -f ${CURRENT_DIR}/src/Dockerfile --build-arg MODEL_NAME=${VOSK_MODEL} .
    fi
    DOCKER_VOSK_CONTAINER_ID=`docker ps -q --filter name=vosk_server_${VOSK_MODEL}`
    if [ ! -z "${DOCKER_VOSK_CONTAINER_ID}" ]; then
        echo "docker container : remove vosk_server_${VOSK_MODEL}"
        docker stop vosk_server_${VOSK_MODEL}
        docker container rm vosk_server_${VOSK_MODEL}
    fi

    docker container run --name vosk_server_${VOSK_MODEL} -d -p ${SERVER_PORT}:2700 vosk:${VOSK_MODEL}
    docker start vosk_server_${VOSK_MODEL}
    ##------------------------------------------

    echo "------------------------------------------"
    echo "VOSK_MODEL  : ${VOSK_MODEL}"
    echo "SERVER_PORT : ${SERVER_PORT}"
    echo "------------------------------------------"
else
    echo "please install docker"
fi
