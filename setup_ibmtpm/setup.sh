#!/bin/bash

# ==================================================================================================
# Parameters
download_dir="/home/user/Downloads"
# ==================================================================================================

echo -e "\n====================================================\n${GREEN}ISetup IBMTPM${NC}\n====================================================\n"

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
