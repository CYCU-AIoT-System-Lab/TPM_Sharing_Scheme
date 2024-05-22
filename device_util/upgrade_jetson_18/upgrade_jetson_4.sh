#!/bin/bash
set -x

sudo apt update
sudo apt upgrade -y
sudo apt autoremove -y

# Remove Jtop warning
sudo rm -rf /usr/shared/vulkan/icd.d

# Remove circular symlink
sudo rm /usr/share/applications/vpil_demos

# Remove distorted nvidia logo in top bar
cd /usr/share/nvpmodel_indicator
sudo mv nv_logo.svg no_logo.svg

# Enable NVIDIA repositories
sudo sed -i 's/# deb/deb/g' /etc/apt/sources.list.d/cuda-l4t-10-2-local.list
sudo sed -i 's/# deb/deb/g' /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
sudo sed -i 's/# deb/deb/g' /etc/apt/sources.list.d/visionworks-repo.list
sudo sed -i 's/# deb/deb/g' /etc/apt/sources.list.d/visionworks-sfm-repo.list
sudo sed -i 's/# deb/deb/g' /etc/apt/sources.list.d/visionworks-tracking-repo.list

# Install GCC 8 (older but required)
sudo apt install gcc-8 g++-8

# Set GCC selector (change to 8)
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8

# Set G++ selector (change to 8)
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 9
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8

# Force overwrite to enable NVIDIA updates
sudo dpkg -i --force-overwrite /var/cache/apt/archives/nvidia-l4t-init*.deb

# Final update
sudo apt update
sudo apt upgrade -y

# Perform required reboot
echo ""
read -rsn1 -p "Press any key to perform required reboot ..."
reboot
