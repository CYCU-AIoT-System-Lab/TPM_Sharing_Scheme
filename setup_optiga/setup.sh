#!/bin/bash

source "../common/function.sh"
source "./function_optiga.sh"
load_preset "./config.ini"

echo_notice "setup_optiga" "setup" "Cloning optiga-tpm-explorer..."
git clone "$optiga_url"

echo_notice "setup_optiga" "setup" "Cancelling auto reboot..."
cd ./optiga-tpm-explorer
sed -i 's/sudo reboot/#sudo reboot/g' ./installation_script.sh

boot_config="/boot/config.txt"
echo_notice "setup_optiga" "setup" "Configuring $boot_config..."
sudo sed -i '70 i \

' $boot_config
sudo sed -i '71 i # Enable TPM' $boot_config
sudo sed -i '72 i dtoverlay=tpm-slb9670' $boot_config

echo_notice "setup_optiga" "setup" "Running optiga-tpm-explorer installation script..."
bash ./installation_script.sh

echo_notice "setup_optiga" "setup" "Reboot to finish installation"
clear_preset
