# =====================================================================================================
# The following code is modified from the following source:
# =====================================================================================================
# Related Source:
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/14
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./tpm_daemen_starter.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

echo -e "\n=====================================================================================================\nStart installing tpm_daemen_starter ...\n=====================================================================================================\n"

# Manipulate services
# 1. tpm2-abrmd.service
# 2. tpm-server.service

systemctl daemon-reload

# Stop tpm2-abrmd.service
systemctl start tpm2-abrmd.service

# Stop tpm-server.service
systemctl start tpm-server.service

# Message
echo -e "\n=====================================================================================================\nAll TPM daemons are started.\n=====================================================================================================\n"

# Show daemon status
echo -e "\n=====================================================================================================\nShow daemon status ...\n=====================================================================================================\n"
echo -e "Press \"q\" to continue script ...\n"
# systemctl status tpm2-abrmd.service
# systemctl status tpm-server.service
service tpm2-abrmd status
service tpm-server status

# Message
echo -e "\n=====================================================================================================\nAll TPM daemons are started.\n=====================================================================================================\n"
