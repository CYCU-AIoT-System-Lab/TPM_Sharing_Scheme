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

# Port number
port=2321

echo -e "\n=====================================================================================================\nStart installing tpm_daemen_starter ...\n=====================================================================================================\n"

# Manipulate services
# 1. tpm2-abrmd.service
# 2. tpm-server.service

# Start tpm-server.service
systemctl daemon-reload
systemctl start tpm-server.service

# Start tpm2-abrmd.service
kill -9 $(lsof -t -i:$port)
systemctl daemon-reload
systemctl start tpm2-abrmd.service

# Message
echo -e "\n=====================================================================================================\nAll TPM daemons are started.\n=====================================================================================================\n"

# Show daemon status
echo -e "\n=====================================================================================================\nShow daemon status ...\n=====================================================================================================\n"
echo -e "Press \"q\" to continue script ...\n"
# systemctl status tpm-server.service
# systemctl status tpm2-abrmd.service
service tpm-server status
service tpm2-abrmd status

# Message
echo -e "\n=====================================================================================================\nAll TPM daemons are started.\n=====================================================================================================\n"
