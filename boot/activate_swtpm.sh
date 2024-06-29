#!/bin/bash

# wait till SWTPM is fully loaded and up
# this should be done with systemd, so that next service
# depended will also be waiting
#sleep 1

SERVER_PORT=2321

#> SWTPM socket interface activation
export TPM2TOOLS_TCTI="swtpm:port=$SERVER_PORT" 
tpm2_startup -c
