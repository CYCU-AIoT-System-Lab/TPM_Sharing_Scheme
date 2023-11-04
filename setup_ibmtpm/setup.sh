#!/bin/bash

# ==================================================================================================
# Parameters
download_dir="/home/user/Downloads"
# ==================================================================================================

BLUE='\033[0;34m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "\n====================================================\n${GREEN}Setup IBMTPM${NC}\n====================================================\n"

echo -e "\n${BLUE}Copying files to ${download_dir}...${NC}"
cp ./*.sh "${download_dir}"

echo -e "\n${BLUE}Changing permissions...${NC}"
chmod +x "${download_dir}/install_req.sh"
chmod +x "${download_dir}/install_ibmswtpm.sh"
chmod +x "${download_dir}/install_ibmtpm2tss.sh"
chmod +x "${download_dir}/install_ibmacs.sh"

echo -e "\n${BLUE}Installing IBMTPM...${NC}"
bash "${download_dir}/install_req.sh"
bash "${download_dir}/install_ibmswtpm.sh"
bash "${download_dir}/install_ibmtpm2tss.sh"
bash "${download_dir}/install_ibmacs.sh"

echo -e "\n====================================================\n${GREEN}Setup IBMTPM Complete${NC}\n====================================================\n"
