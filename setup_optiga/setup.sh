#!/bin/bash

source "../common/functions.sh"
source "./function_optiga.sh"
load_preset "./config.ini"

dirname="setup_optiga"
filename="setup"

echo_notice "${dirname}" "${filename}" "Cloning optiga-tpm-explorer..."
err_retry_exec "git clone $optiga_url" 1 5 "${dirname}" "${filename}" 1

echo_notice "${dirname}" "${filename}" "Cancelling auto reboot..."
cd ./optiga-tpm-explorer
sed -i 's/sudo reboot/#sudo reboot/g' ./installation_script.sh # Disable auto reboot

echo_notice "${dirname}" "${filename}" "Remove default libssl-dev installation..."
sed -i 's/libssl-dev//g' ./installation_script.sh

if [ $install_platform -eq 3 ]; then
    boot_config="/boot/config.txt"
    echo_notice "${dirname}" "${filename}" "Configuring $boot_config..."
    sudo sed -i '70 i \

    ' $boot_config
    sudo sed -i '71 i # Enable TPM' $boot_config
    sudo sed -i '72 i dtoverlay=tpm-slb9670' $boot_config
elif [ $install_platform -eq 4 ]; then
    boot_config="/boot/firmware/config.txt"
    echo_notice "${dirname}" "${filename}" "Configuring $boot_config..."
    sudo bash -c "echo -e \"\n# Enable TPM\ndtoverlay=tpm-slb9670\" >> $boot_config"
else
    echo_warn "${dirname}" "${filename}" "Invalid install_platform: $install_platform, skipping TPM enabling..."
fi

echo_notice "${dirname}" "${filename}" "Running optiga-tpm-explorer installation script..."
bash ./installation_script.sh

echo_notice "${dirname}" "${filename}" "${RED}${BOLD}Reboot to finish installation${END}"
clear_preset
