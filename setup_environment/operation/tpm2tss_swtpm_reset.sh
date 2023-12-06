# =====================================================================================================
# The following code is modified from the following source:
# 1. https://github.com/stefanberger/swtpm/wiki/Using-the-Intel-TSS-with-swtpm
# =====================================================================================================
# Related Source:
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/13
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./tpm2tss_swtpm_start.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

# info
echo -e "\nPlz execute \"../activate/activate_swtpm_simulator.sh\" and \"./tpm2tss_swtpm_start.sh\" first.\n"

# create a directory for swtpm
swtpm_ioctl -i --tcp :2322

# message
echo -e "\n Finished! \n"
