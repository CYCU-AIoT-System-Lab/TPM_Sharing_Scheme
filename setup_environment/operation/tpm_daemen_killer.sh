# =====================================================================================================
# The following code is modified from the following source:
# 1. https://francislampayan.medium.com/how-to-setup-tpm-simulator-in-ubuntu-20-04-25ec673b88dc
# 2. https://gist.github.com/fbdlampayan/5ceaadda9c32d4c23478ee46f80207f9#file-gistfile1-txt
# =====================================================================================================
# Related Source:
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/14
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./tpm_daemen_killer.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

echo -e "\n=====================================================================================================\nStart installing tpm_daemen_killer ...\n=====================================================================================================\n"

# Manipulate services
# 1. tpm2-abrmd.service
# 2. tpm-server.service

# Stop tpm2-abrmd.service
systemctl stop tpm2-abrmd.service

# Stop tpm-server.service
systemctl stop tpm-server.service

# Show daemon status
echo -e "\n=====================================================================================================\nShow daemon status ...\n=====================================================================================================\n"
# systemctl status tpm2-abrmd.service
# systemctl status tpm-server.service
service tpm2-abrmd status
service tpm-server status

# Message
echo -e "\n=====================================================================================================\nAll TPM daemons are killed.\n=====================================================================================================\n"
