#!/bin/bash

# ==================================================================================================
# Parameters
base_dir="/opt"
ibmacs_ver="1658"
acsMode=1 # 1: Server, 2: Client
# ==================================================================================================

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
NC='\033[0m'

echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMACS${NC}\n====================================================\n"

path_ibmtss="${base_dir}/ibmtss${ibmtss_ver}/"
path_ibmacs="${base_dir}/ibmacs${ibmacs_ver}/"

echo -e "${BOLD}${BLUE}Installing IBMACS dependencies ......${NC}"
if [ $acsMode == 1 ]; then
    # for Server
    apt-get install -y libjson-c-dev apache2 php php-dev php-mysql mysql-server libmysqlclient-dev libssl-dev
elif [ $acsMode == 2 ]; then
    # for Client
    apt-get install -y libjson-c-dev libssl-dev
else 
    echo -e "${BOLD}${RED}Invalid acsMode${NC}"
    exit 1
fi

if [ $acsMode == 1 ]; then
    echo -e "${BOLD}${BLUE}Setting database ......${NC}"
    cd "${path_ibmacs}/acs/"
    mysql -Bse "CREATE DATABASE IF NOT EXISTS tpm2;"
    mysql -Bse "CREATE USER IF NOT EXISTS 'tpm2ACS'@'localhost' IDENTIFIED BY '123456';"
    mysql -Bse "GRANT ALL PRIVILEGES ON tpm2.* TO 'tpm2ACS'@'localhost';"
    mysql -D tpm2 < "${path_ibmacs}/acs/dbinit.sql"
fi

echo -e "${BOLD}${BLUE}Setting path ......${NC}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"
export PATH="${PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"

echo -e "${BOLD}${BLUE}Creating symbolic link to ${path_ibmacs} ......${NC}"
ln -s "${path_ibmacs}/acs" "${base_dir}/ibmacs"
