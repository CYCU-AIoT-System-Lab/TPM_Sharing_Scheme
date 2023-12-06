# =====================================================================================================
# The following code is modified from the following source:
# 1. https://francislampayan.medium.com/how-to-setup-tpm-simulator-in-ubuntu-20-04-25ec673b88dc
# 2. https://gist.github.com/fbdlampayan/5ceaadda9c32d4c23478ee46f80207f9#file-gistfile1-txt
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/11
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./TPM-simulator_installation.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

echo -e "\nList of test commands:\n"

# TPM-simulator installation
# command: service tpm-server status
echo -e "\n1. TPM-simulator installation"
echo -e "command: \"service tpm-server status\""

# tpm2-abrmd installation
# command: service tpm2-abrmd status
echo -e "\n2. tpm2-abrmd installation"
echo -e "command: \"service tpm2-abrmd status\""

# tpm2-tools installation (test with openssl)
# command: openssl rand -engine tpm2tss -hex 20
echo -e "\n3. tpm2-tools installation"
echo -e "command: \"openssl rand -engine tpm2tss -hex 20\""

# tpm2-tools isntallation
# command: tpm2_pcrread
echo -e "\n4. tpm2-tools isntallation"
echo -e "command: \"tpm2_pcrread\"\n"
