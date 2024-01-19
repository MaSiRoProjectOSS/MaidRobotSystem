#!/bin/bash

MRS_CONFIG=${1:-$MRS_CONFIG}
ENV_FILE=$(readlink -f $(dirname $0)/../env.sh)
source ${ENV_FILE} ${MRS_CONFIG}

echo "MRS_CONFIG=${MRS_CONFIG}"
echo "MRS_CAST_NAME=${MRS_CAST_NAME}"
