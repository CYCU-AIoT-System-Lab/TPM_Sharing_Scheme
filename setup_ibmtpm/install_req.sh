#!/bin/bash

download_dir="/home/user/Downloads"
ibmtss_ver="2.1.1"
ibmtpm_ver="1682"
ibmacs_ver="1658"

echo -e "\n====================================================\nInstalling IBMTPM\n====================================================\n"

echo -e "Updating system..."
apt-get update
apt-get upgrade -y

echo -e "Installing tools..."
apt-get install -y htop iftop neovim git curl wget

echo -e "Installing IBMTPM dependencies..."
apt-get install -y build-essential make gcc libssl-dev

echo -e "Downloading IBMTPM..."
file_ibmtss="${download_dir}/ibmtss${ibmtss_ver}.tar.gz"
file_ibmtpm="${download_dir}/ibmtpm${ibmtpm_ver}.tar.gz"
file_ibmacs="${download_dir}/ibmacs${ibmacs_ver}.tar.gz"
curl -L https://sourceforge.net/project/ibmtpm20tss/files/ibmtss${ibmtss_ver}.tar.gz > ${file_ibmtss}
curl -L https://sourceforge.net/project/ibmswtpm2/files/ibmtpm${ibmtpm_ver}.tar.gz > ${file_ibmtpm}
curl -L https://sourceforge.net/projects/ibmtpm20acs/files/ibmacs${ibmacs_ver}.tar.gz > ${file_ibmacs}

echo -e "Creating directories..."
path_ibmtss="/opt/ibmtss${ibmtss_ver}/"
path_ibmtpm="/opt/ibmtpm${ibmtpm_ver}/"
path_ibmacs="/opt/ibmacs${ibmacs_ver}/"
mkdir "${path_ibmtss}"
mkdir "${path_ibmtpm}"
mkdir "${path_ibmacs}"

echo -e "Copying files to /opt..."
cp ${file_ibmtss} ${path_ibmtss}
cp ${file_ibmtpm} ${path_ibmtpm}
cp ${file_ibmacs} ${path_ibmacs}

echo -e "Extracting files..."
tar -zxvf "/opt/ibmtss${ibmtss_ver}/ibmtss${ibmtss_ver}.tar.gz"
tar -zxvf "/opt/ibmtpm${ibmtpm_ver}/ibmtpm${ibmtpm_ver}.tar.gz"
tar -zxvf "/opt/ibmacs${ibmacs_ver}/ibmacs${ibmacs_ver}.tar.gz"

echo -e "\n====================================================\nInstalling IBMTPM Complete\n====================================================\n"
