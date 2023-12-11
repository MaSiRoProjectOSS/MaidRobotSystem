#!/bin/bash

ENV_FILE=$(readlink -f $(dirname $0)/../env.sh)
source ${ENV_FILE}
