#!/bin/bash
set -x

# Ref: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html

echo "> Update apt ..."
sudo apt remove --purge -y chromium-browser chromium-browser-l10n
sudo apt update
sudo apt install -y htop iftop net-tools
sudo apt upgrade -y
sudo apt autoremove -y
sudo sed -i '/Prompt=never/c\Prompt=normal' /etc/update-manager/release-upgrades
sudo apt update
sudo apt dist-upgrade -y

echo "> Install apt-fast ..."
sudo add-apt-repository ppa:apt-fast/stable -y
sudo apt-get install apt-fast
echo "alias apt-get='apt-fast'" >> ~/.bash_aliases

# Perform required reboot
echo ""
read -rsn1 -p "Press any key to perform required reboot ..."
reboot
