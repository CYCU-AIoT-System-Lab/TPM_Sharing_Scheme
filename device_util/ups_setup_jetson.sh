#!/bin/bash

# Ref: https://www.waveshare.com/ups-power-module.htm
# Ref: --> https://blog.cavedu.com/2019/10/04/nvidia-jetson-nano-fan/

install_dir="/opt"

echo "> Updating apt ..."
sudo apt update && sudo apt upgrade -y

echo "> Installing from debian package ..."
sudo apt-get install python3-smbus -y
sudo apt-get install python-smbus -y

echo "> Cloning from waveshare github repository ..."
cd $install_dir
sudo git clone https://github.com/waveshare/UPS-Power-Module

echo "> Final installing ..."
cd $install_dir/UPS-Power-Module
sudo ./install.sh

echo "> Installation finished!"
