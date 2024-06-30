#!/bin/bash

source "../common/functions.sh"
source "./function_boot.sh"
load_preset "./config.ini"

#> SWTPM socket interface startup
mkdir /tmp/myvtpm
swtpm socket --tpmstate dir=/tmp/myvtpm --tpm2 --ctrl type=tcp,port=$SWTPM_CTRL_PORT --server type=tcp,port=$SWTPM_SERVER_PORT --flags not-need-init
