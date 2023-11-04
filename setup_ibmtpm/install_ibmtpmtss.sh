#!/bin/bash

# ==================================================================================================
# Parameters
base_dir="/opt"
ibmtss_ver="2.1.1"
# ==================================================================================================

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
NC='\033[0m'

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMTSS${NC}\n====================================================\n"

path_ibmtss="${base_dir}/ibmtss${ibmtss_ver}/"

echo -e "${BOLD}${BLUE}Compiling IBMTSS ......${NC}"
cd "${path_ibmtss}/src"
make

echo -e "${BOLD}${BLUE}Creating symbolic link to ${path_ibmtss} ......${NC}"
ln -s "${path_ibmtss}" "${base_dir}/ibmtss"

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMTSS Complete${NC}\n====================================================\n"
