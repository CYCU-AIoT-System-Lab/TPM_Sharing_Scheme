#!/bin/bash

source "../common/functions.sh"
source "./function_boot.sh"
load_preset "./config.ini"

#> SWTPM socket interface startup
cd $BASE_DIR/ibmtpm/src
sudo mkdir $SWTPM_SOCKET_DEVICE || :
swtpm socket --tpmstate dir=$SWTPM_SOCKET_DEVICE --tpm2 --ctrl type=tcp,port=$SWTPM_CTRL_PORT --server type=tcp,port=$SWTPM_SERVER_PORT --flags not-need-init
