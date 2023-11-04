#!/bin/bash

# ==================================================================================================
# Parameters
download_dir="/home/user/Downloads"
# ==================================================================================================

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
NC='\033[0m'

echo -e "\n====================================================\n${BOLD}${GREEN}Setup IBMTPM${NC}\n====================================================\n"

echo -e "${BOLD}${BLUE}Copying files to ${download_dir} ......${NC}"
cp ./*.sh "${download_dir}"

echo -e "${BOLD}${BLUE}Changing permissions ......${NC}"
chmod +x "${download_dir}/install_req.sh"
chmod +x "${download_dir}/install_ibmswtpm.sh"
chmod +x "${download_dir}/install_ibmtpm2tss.sh"
chmod +x "${download_dir}/install_ibmacs.sh"

echo -e "${BOLD}${BLUE}Installing IBMTPM ......${NC}"
bash "${download_dir}/install_req.sh"
bash "${download_dir}/install_ibmswtpm.sh"
bash "${download_dir}/install_ibmtpm2tss.sh"
bash "${download_dir}/install_ibmacs.sh"

echo -e "\n====================================================\n${BOLD}${GREEN}Setup IBMTPM Complete${NC}\n====================================================\n"
