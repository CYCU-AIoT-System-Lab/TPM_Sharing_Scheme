#!/bin/bash

source "../common/function.sh"
source "./function_optiga.sh"
load_preset "../config.ini"

echo_notice "common" "setup" "Cloning optiga-tpm-explorer..."
git clone "$optiga_url"

echo_notice "common" "setup" "Cancelling auto reboot..."
cd ./optiga-tpm-explorer
sed -i 's/sudo reboot/#sudo reboot/g' ./installation_script.sh

echo_notice "common" "setup" "Running optiga-tpm-explorer installation script..."
bash ./installation_script.sh

echo_notice "common" "setup" "Reboot to finish installation"
clear_preset
