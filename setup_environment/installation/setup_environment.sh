# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/11
# =====================================================================================================
# Usage:
# 1. Acquire all required files from the following source:
#       https://github.com/CYCU-AIoT-System-Lab/TPM_Sharing_Scheme/tree/main/setup_environment
# 2. Nevigate to your desired directory and run this script with "sudo ./setup_environment.sh"
# 3. Recommend to move all scripts to a different directory than downloaded repository.
# =====================================================================================================
#!/bin/bash

# update and upgrade
apt-get update
apt-get upgrade -y

# install dependencies for fresh installation
apt-get install -y \
htop \
iftop \
neovim \
git \
wget \
curl \
build-essential

# change bash scripts to executable
chmod +x ./github-desktop_installation.sh
chmod +x ./TPM-simulator_installation.sh
chmod +x ./tpm2-tss_installation.sh
chmod +x ./tpm2-abrmd_installation.sh
chmod +x ./tpm2-tss-engine_installation.sh
chmod +x ./tpm2-tools_installation.sh
chmod +x ./swtpm-simulator_installation.sh
chmod +x ./test_commands.sh

# call bash scripts one by one
# bash ./github-desktop_installation.sh -y
bash ./TPM-simulator_installation.sh
bash ./tpm2-tss_installation.sh
bash ./tpm2-abrmd_installation.sh
bash ./tpm2-tss-engine_installation.sh
bash ./tpm2-tools_installation.sh
bash ./swtpm-simulator_installation.sh

echo -e "Installation completed.\n"

bash ./test_commands.sh

# message
echo -e "\n ===================================================================================================== \n Installation process is fully finished! \n Please reboot your system. \n ===================================================================================================== \n"
