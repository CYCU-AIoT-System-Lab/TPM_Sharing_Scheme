#!/bin/bash

# ==================================================================================================
# Parameters
base_dir="/opt"
ibmtpm_ver="1682"
# ==================================================================================================

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
NC='\033[0m'

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMTPM${NC}\n====================================================\n"

path_ibmtpm="${base_dir}/ibmtpm${ibmtpm_ver}/"

echo -e "${BOLD}${BLUE}Cleaning up ......${NC}"
cd "${path_ibmtpm}/src/"
make clean

echo -e "${BOLD}${BLUE}Compiling IBMTPM ......${NC}"
cd "${path_ibmtpm}/src/"
make

echo -e "${BOLD}${BLUE}Creating symbolic link to ${path_ibmtpm} ......${NC}"
ln -s "${path_ibmtpm}" "${base_dir}/ibmtpm"

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMTPM Complete${NC}\n====================================================\n"
