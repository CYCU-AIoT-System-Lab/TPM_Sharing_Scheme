# =====================================================================================================
# The following code is modified from the following source:
# 1. https://github.com/stefanberger/swtpm/wiki/Using-the-Intel-TSS-with-swtpm
# =====================================================================================================
# Related Source:
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/13
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./activate_swtpm_simulator.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

# port number
port=2321

# info
echo -e "\nIf this script ends with error message:\n\"swtpm: Could not open TCP socket: Address already in use\"\nTry running \"../operation/tpm_daemen_killer\".\nIf not, plz keep it running.\n"

# kill all tpm daemons
kill -9 $(lsof -t -i:$port)

# create a directory for swtpm
dir=/tmp/tpm0
ctrl_port=2322
server_port=$port
mkdir $dir
swtpm socket --tpmstate dir=$dir --tpm2 --ctrl type=tcp,port=$ctrl_port --server type=tcp,port=$server_port --flags not-need-init

# Go to "./operation/tpm2tss_swtpm_start.sh"
