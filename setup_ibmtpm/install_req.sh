#!/bin/bash

# ==================================================================================================
# Parameters
download_dir="/home/user/Downloads"
base_dir="/opt"
ibmtss_ver="1.6.0" # latest: 2.1.1
ibmtpm_ver="1637" # latest: 1682
ibmacs_ver="1658" # latest: 1658
# ==================================================================================================

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
NC='\033[0m'

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing requirements${NC}\n====================================================\n"

fn_ibmtss="ibmtss${ibmtss_ver}.tar.gz"
fn_ibmtpm="ibmtpm${ibmtpm_ver}.tar.gz"
fn_ibmacs="ibmacs${ibmacs_ver}.tar.gz"
file_ibmtss="${download_dir}/${fn_ibmtss}"
file_ibmtpm="${download_dir}/${fn_ibmtpm}"
file_ibmacs="${download_dir}/${fn_ibmacs}"
path_ibmtss="${base_dir}/ibmtss${ibmtss_ver}/"
path_ibmtpm="${base_dir}/ibmtpm${ibmtpm_ver}/"
path_ibmacs="${base_dir}/ibmacs${ibmacs_ver}/"

echo -e "${BOLD}${BLUE}Updating system ......${NC}"
apt-get update
apt-get upgrade -y

echo -e "${BOLD}${BLUE}Installing tools ......${NC}"
apt-get install -y htop iftop neovim git curl wget

echo -e "${BOLD}${BLUE}Installing IBMTPM dependencies ......${NC}"
apt-get install -y build-essential make gcc libssl-dev libtss0 libtss-dev libtss2-dev libtss2-doc libtss2-esys0 libtss2-esys-3.0.2-0 libtss2-fapi1 libtss2-mu0 libtss2-policy0 libtss2-rc0 libtss2-sys1 libtss2-tcti-cmd0 libtss2-tcti-device0 libtss2-tcti-libtpms0 libtss2-tcti-mssim0 libtss2-tcti-pcap0 libtss2-tcti-spi-helper0 libtss2-tcti-swtpm0 libtss2-tcti-tabrmd-dev libtss2-tcti-tabrmd0 libtss2-tctidr0

echo -e "${BOLD}${BLUE}Downloading IBMTPM ......${NC}"
wget "https://sourceforge.net/projects/ibmtpm20tss/files/${fn_ibmtss}/download" -O ${file_ibmtss}
wget "https://sourceforge.net/projects/ibmswtpm2/files/${fn_ibmtpm}/download" -O ${file_ibmtpm}
wget "https://sourceforge.net/projects/ibmtpm20acs/files/${fn_ibmacs}/download" -O ${file_ibmacs}

echo -e "${BOLD}${BLUE}Creating directories ......${NC}"
mkdir "${path_ibmtss}"
mkdir "${path_ibmtpm}"
mkdir "${path_ibmacs}"

echo -e "${BOLD}${BLUE}Copying files to /opt ......${NC}"
cp ${file_ibmtss} ${path_ibmtss}
cp ${file_ibmtpm} ${path_ibmtpm}
cp ${file_ibmacs} ${path_ibmacs}

echo -e "${BOLD}${BLUE}Extracting files ......${NC}"
tar -zxvf "${path_ibmtss}/${fn_ibmtss}" -C ${path_ibmtss}
tar -zxvf "${path_ibmtpm}/${fn_ibmtpm}" -C ${path_ibmtpm}
tar -zxvf "${path_ibmacs}/${fn_ibmacs}" -C ${path_ibmacs}

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing requirements Complete${NC}\n====================================================\n"
