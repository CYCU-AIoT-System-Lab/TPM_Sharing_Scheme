#!/bin/bash

# ==================================================================================================
# Parameters
base_dir="/opt"
ibmtss_ver="2.1.1"
verMode=1 # 1: TPM 2.0, 2: TPM 1.2 & 2.0
TPMMode=2 # 1: Pysical TPM, 2: Software TPM
# ==================================================================================================

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
NC='\033[0m'

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMTSS${NC}\n====================================================\n"

path_ibmtss="${base_dir}/ibmtss${ibmtss_ver}/"

echo -e "${BOLD}${BLUE}Cleaning up ......${NC}"
if [ $verMode == 1 ]; then
    # for TPM 2.0
    cd "${path_ibmtss}/utils/"
    make -f clean
elif [ $verMode == 2]; then
    # for TPM 1.2 & 2.0
    cd "${path_ibmtss}/utils/"
    make -f clean
    cd "${path_ibmtss}/utils12/"
    make -f clean
else 
    echo -e "${BOLD}${RED}Invalid verMode${NC}"
    exit 1
fi

echo -e "${BOLD}${BLUE}Compiling IBMTSS ......${NC}"
if [ $verMode == 1 ]; then
    # for TPM 2.0
    cd "${path_ibmtss}/utils/"
    make -f makefiletpm20
elif [ $verMode == 2]; then
    # for TPM 1.2 & 2.0
    cd "${path_ibmtss}/utils/"
    make -f makefiletpmc
    cd "${path_ibmtss}/utils12/"
    make -f makefiletpmc
else 
    echo -e "${BOLD}${RED}Invalid verMode${NC}"
    exit 1
fi

echo -e "${BOLD}${BLUE}Setting path ......${NC}"
if [ $TPMMode == 1 ]; then
    # for Pysical TPM
    export TPM_INTERFACE_TYPE=dev
elif [ $TPMMode == 2 ]; then
    # for Software TPM
    export TPM_INTERFACE_TYPE=socsim
else 
    echo -e "${BOLD}${RED}Invalid TPMMode${NC}"
    exit 1
fi

echo -e "${BOLD}${BLUE}Creating symbolic link to ${path_ibmtss} ......${NC}"
ln -s "${path_ibmtss}" "${base_dir}/ibmtss"

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMTSS Complete${NC}\n====================================================\n"
