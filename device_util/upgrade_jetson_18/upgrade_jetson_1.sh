#!/bin/bash
set -x
script=$(realpath "$0")
script_path=$(dirname "$script")

# Ref: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html

echo "> Install apt-fast ..."
echo "choose \"apt-get\" and the maximum number of connections"
sudo add-apt-repository ppa:apt-fast/stable -y
sudo apt-get install -y apt-fast htop iftop net-tools tmux
echo "alias apt-get='apt-fast'" >> ~/.bash_aliases
echo "alias apt='apt-fast'" >> ~/.bash_aliases
source ~/.bash_aliases

echo "> Update apt ..."
sudo apt remove --purge -y chromium-browser chromium-browser-l10n
sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y
sudo sed -i '/Prompt=never/c\Prompt=normal' /etc/update-manager/release-upgrades
sudo apt update
sudo apt dist-upgrade -y

echo "> Creating progress flag file ..."
touch $script_path/upgrade_jetson_1_done.flag

# Perform required reboot
echo ""
read -rsn1 -p "Press any key to perform required reboot ..."
reboot
