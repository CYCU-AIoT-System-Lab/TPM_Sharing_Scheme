#!/bin/bash
sudo apt-get update
sudo apt-get install -y git
sudo apt autoremove -y
ssh-keyscan github.com >> ~/.ssh/known_hosts
git config --global user.email "dachuan516@gmail.com"
git config --global user.name  "belongtothenight"
git config --global core.compression 0
git clone git@github.com:CYCU-AIoT-System-Lab/TPM_Sharing_Scheme.git
chmod +x ./TPM_Sharing_Scheme/common/setup.sh
echo "cd ./TPM_Sharing_Scheme/common"
echo "./setup.sh"
