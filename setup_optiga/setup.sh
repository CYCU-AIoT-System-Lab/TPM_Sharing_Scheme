#!/bin/bash

git_url="https://github.com/Infineon/optiga-tpm-explorer.git"

git clone $git_url
cd optiga-tpm-explorer
sed -i 's/sudo reboot/#sudo reboot/g' ./installation_script.sh
bash ./installation_script.sh

echo "Reboot to finish installation"
