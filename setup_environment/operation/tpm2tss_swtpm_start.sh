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
echo -e "\nPlz execute \"../activate/activate_swtpm_simulator.sh\" first.\n"
echo -e "\nIf this script ends with output like:\nsha1:\n  10: 0x0000000000000000000000000000000000000000\nSW TPM simulator is successfully activated."

# activate swtpm
export TPM2TOOLS_TCTI="swtpm:port=2321"
tpm2_startup -c
tpm2_pcrread sha1:10

# message
echo -e "\n Finished! \n"
