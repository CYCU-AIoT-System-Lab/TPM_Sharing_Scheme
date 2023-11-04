#!/bin/bash

download_dir="/home/user/Downloads"
ibmtss_ver="2.1.1"
ibmtpm_ver="1682"
ibmacs_ver="1658"

echo -e "\n====================================================\nInstalling IBMTPM\n====================================================\n"

BLUE='\033[0;34m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${BLUE}Updating system...${NC}"
apt-get update
apt-get upgrade -y

echo -e "${BLUE}Installing tools...${NC}"
apt-get install -y htop iftop neovim git curl wget

echo -e "${BLUE}Installing IBMTPM dependencies...${NC}"
apt-get install -y build-essential make gcc libssl-dev

echo -e "${BLUE}Downloading IBMTPM...${NC}"
file_ibmtss="${download_dir}/ibmtss${ibmtss_ver}.tar.gz"
file_ibmtpm="${download_dir}/ibmtpm${ibmtpm_ver}.tar.gz"
file_ibmacs="${download_dir}/ibmacs${ibmacs_ver}.tar.gz"
wget "https://sourceforge.net/projects/ibmswtpm2/files/ibmtpm${ibmtpm_ver}.tar.gz/download" -O ${file_ibmtpm}
wget "https://sourceforge.net/projects/ibmswtpm2/files/ibmtss${ibmtss_ver}.tar.gz/download" -O ${file_ibmtss}
wget "https://sourceforge.net/projects/ibmtpm20acs/files/ibmacs${ibmacs_ver}.tar.gz/download" -O ${file_ibmacs}

echo -e "${BLUE}Creating directories...${NC}"
path_ibmtss="/opt/ibmtss${ibmtss_ver}/"
path_ibmtpm="/opt/ibmtpm${ibmtpm_ver}/"
path_ibmacs="/opt/ibmacs${ibmacs_ver}/"
mkdir "${path_ibmtss}"
mkdir "${path_ibmtpm}"
mkdir "${path_ibmacs}"

echo -e "${BLUE}Copying files to /opt...${NC}"
cp ${file_ibmtss} ${path_ibmtss}
cp ${file_ibmtpm} ${path_ibmtpm}
cp ${file_ibmacs} ${path_ibmacs}

echo -e "${BLUE}Extracting files...${NC}"
tar -zxvf "/opt/ibmtss${ibmtss_ver}/ibmtss${ibmtss_ver}.tar.gz"
tar -zxvf "/opt/ibmtpm${ibmtpm_ver}/ibmtpm${ibmtpm_ver}.tar.gz"
tar -zxvf "/opt/ibmacs${ibmacs_ver}/ibmacs${ibmacs_ver}.tar.gz"

echo -e "\n====================================================\nInstalling IBMTPM Complete\n====================================================\n"
