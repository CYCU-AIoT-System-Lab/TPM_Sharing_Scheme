#!/bin/bash

source "../common/functions.sh"
source "./function_optiga.sh"
load_preset "./config.ini"

echo_notice "setup_optiga" "setup" "Cloning optiga-tpm-explorer..."
git clone "$optiga_url"

echo_notice "setup_optiga" "setup" "Cancelling auto reboot..."
cd ./optiga-tpm-explorer
sed -i 's/sudo reboot/#sudo reboot/g' ./installation_script.sh # Disable auto reboot

echo_notice "setup_optiga" "setup" "Remove default libssl-dev installation..."
sed -i 's/libssl-dev//g' ./installation_script.sh

if [ $install_platform -eq 3 ]; then
    boot_config="/boot/config.txt"
    echo_notice "setup_optiga" "setup" "Configuring $boot_config..."
    sudo sed -i '70 i \

    ' $boot_config
    sudo sed -i '71 i # Enable TPM' $boot_config
    sudo sed -i '72 i dtoverlay=tpm-slb9670' $boot_config
elif [ $install_platform -eq 4 ]; then
    boot_config="/boot/firmware/config.txt"
    echo_notice "setup_optiga" "setup" "Configuring $boot_config..."
    sudo bash -c "echo -e \"\n# Enable TPM\ndtoverlay=tpm-slb9670\" >> $boot_config"
else
    echo_warn "setup_optiga" "setup" "Invalid install_platform: $install_platform, skipping TPM enabling..."
fi

echo_notice "setup_optiga" "setup" "Running optiga-tpm-explorer installation script..."
bash ./installation_script.sh

echo_notice "setup_optiga" "setup" "Reboot to finish installation"
clear_preset
