#!/bin/bash

# ==================================================================================================
# Parameters
download_dir="/home/user/Downloads"
ibmtss_ver="2.1.1"
ibmtpm_ver="1682"
ibmacs_ver="1658"
# ==================================================================================================

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
NC='\033[0m'

echo -e "\n====================================================\n${BOLD}${GREEN}Installing IBMTPM${NC}\n====================================================\n"

fn_ibmtss="ibmtss${ibmtss_ver}.tar.gz"
fn_ibmtpm="ibmtpm${ibmtpm_ver}.tar.gz"
fn_ibmacs="ibmacs${ibmacs_ver}.tar.gz"
file_ibmtss="${download_dir}/${fn_ibmtss}"
file_ibmtpm="${download_dir}/${fn_ibmtpm}"
file_ibmacs="${download_dir}/${fn_ibmacs}"
path_ibmtss="/opt/ibmtss${ibmtss_ver}/"
path_ibmtpm="/opt/ibmtpm${ibmtpm_ver}/"
path_ibmacs="/opt/ibmacs${ibmacs_ver}/"

echo -e "\n${BOLD}${BLUE}Updating system...${NC}"
apt-get update
apt-get upgrade -y

echo -e "\n${BOLD}${BLUE}Installing tools...${NC}"
apt-get install -y htop iftop neovim git curl wget

echo -e "\n${BOLD}${BLUE}Installing IBMTPM dependencies...${NC}"
apt-get install -y build-essential make gcc libssl-dev

echo -e "\n${BOLD}${BLUE}Downloading IBMTPM...${NC}"
wget "https://sourceforge.net/projects/ibmtpm20tss/files/${fn_ibmtss}/download" -O ${file_ibmtss}
wget "https://sourceforge.net/projects/ibmswtpm2/files/${fn_ibmtpm}/download" -O ${file_ibmtpm}
wget "https://sourceforge.net/projects/ibmtpm20acs/files/${fn_ibmacs}/download" -O ${file_ibmacs}

echo -e "\n${BOLD}${BLUE}Creating directories...${NC}"
mkdir "${path_ibmtss}"
mkdir "${path_ibmtpm}"
mkdir "${path_ibmacs}"

echo -e "\n${BOLD}${BLUE}Copying files to /opt...${NC}"
cp ${file_ibmtss} ${path_ibmtss}
cp ${file_ibmtpm} ${path_ibmtpm}
cp ${file_ibmacs} ${path_ibmacs}

echo -e "\n${BOLD}${BLUE}Extracting files...${NC}"
tar -zxvf "${path_ibmtss}/${fn_ibmtss}" -C ${path_ibmtss}
tar -zxvf "${path_ibmtpm}/${fn_ibmtpm}" -C ${path_ibmtpm}
tar -zxvf "${path_ibmacs}/${fn_ibmacs}" -C ${path_ibmacs}

echo -e "\n====================================================\n${BOLD}${GREEN}Installing IBMTPM Complete${NC}\n====================================================\n"
