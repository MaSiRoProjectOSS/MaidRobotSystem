#!/bin/bash

COLOR_ON_BLUE="\e[34m"
COLOR_ON_GREEN="\e[32m"
COLOR_ON_RED="\e[31m"
COLOR_OFF="\e[m"
ENV_DIR="$(cd $(dirname $0); pwd)"

LOCAL_MRS_CONFIG=""
COMMAND=""
while (( $# > 0 ))
do
  case $1 in
    --mrs_config | --mrs_config*)
      if [[ "$1" =~ ^--mrs_config= ]]; then
        LOCAL_MRS_CONFIG=$(echo $1 | sed -e 's/^--mrs_config=//')
      elif [[ "$1" =~ ^--mrs_config ]]; then
        LOCAL_MRS_CONFIG=$(echo $1 | sed -e 's/^--mrs_config//')
        if [ -z "${LOCAL_MRS_CONFIG}" ]; then
          LOCAL_MRS_CONFIG=$(echo $2 | sed -e 's/^--mrs_config//')
          shift
        fi
      else
        LOCAL_MRS_CONFIG=$(echo $2 | sed -e 's/^--mrs_config//')
        shift
      fi
      if [ ! -z "${LOCAL_MRS_CONFIG}" ]; then
        LOCAL_MRS_CONFIG=$(readlink -f ${LOCAL_MRS_CONFIG})
      fi
        ;;
      * ) COMMAND=${COMMAND}" "$1 ;;
  esac
  shift
done

source ${ENV_DIR}/env.sh ${LOCAL_MRS_CONFIG}
echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"
echo -e "${COLOR_ON_BLUE}MRS_CONFIG : ${COLOR_OFF}${MRS_CONFIG}"
echo -e "${COLOR_ON_BLUE}Command    :${COLOR_OFF}${COMMAND}"
echo -e "${COLOR_ON_BLUE}===============================================================================${COLOR_OFF}"

if [ ! -z "${COMMAND}" ]; then
    ${COMMAND}
fi
