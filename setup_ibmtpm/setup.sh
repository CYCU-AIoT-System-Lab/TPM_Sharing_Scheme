#!/bin/bash

# ==================================================================================================
# Parameters
download_dir="/home/user/Downloads"
base_dir="/opt"
ibmtss_ver="2.1.1" # latest: 2.1.1
ibmtpm_ver="1682" # latest: 1682
ibmacs_ver="1658" # latest: 1658
verMode=2 # 1: TPM 2.0, 2: TPM 1.2 & 2.0
TPMMode=2 # 1: Pysical TPM, 2: Software TPM
# ==================================================================================================

BOLD='\033[1m'
BLUE='\033[34m'
RED='\033[31m'
GREEN='\033[32m'
NC='\033[0m'

fn_ibmtss="ibmtss${ibmtss_ver}.tar.gz"
fn_ibmtpm="ibmtpm${ibmtpm_ver}.tar.gz"
fn_ibmacs="ibmacs${ibmacs_ver}.tar.gz"
file_ibmtss="${download_dir}/${fn_ibmtss}"
file_ibmtpm="${download_dir}/${fn_ibmtpm}"
file_ibmacs="${download_dir}/${fn_ibmacs}"
path_ibmtss="${base_dir}/ibmtss${ibmtss_ver}/"
path_ibmtpm="${base_dir}/ibmtpm${ibmtpm_ver}/"
path_ibmacs="${base_dir}/ibmacs${ibmacs_ver}/"

echo -e "\n====================================================\n>>${BOLD}${GREEN}Setup${NC}\n====================================================\n"

echo -e "${BOLD}${BLUE}Copying files to ${download_dir} ......${NC}"
cp ./*.sh "${download_dir}"

# echo -e "${BOLD}${BLUE}Changing permissions ......${NC}"
# chmod +x "${download_dir}/install_req.sh"
# chmod +x "${download_dir}/install_ibmtpmtss.sh"
# chmod +x "${download_dir}/install_ibmswtpm.sh"
# chmod +x "${download_dir}/install_ibmacs.sh"

# echo -e "${BOLD}${BLUE}Installing IBMTPM ......${NC}"
# bash "${download_dir}/install_req.sh"
# bash "${download_dir}/install_ibmtpmtss.sh"
# bash "${download_dir}/install_ibmswtpm.sh"
# bash "${download_dir}/install_ibmacs.sh"

install_req
install_ibmtpmtss
install_ibmswtpm
install_ibmacs

echo -e "\n====================================================\n>>${BOLD}${GREEN}Setup Complete${NC}\n====================================================\n"

install_req () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing requirements${NC}\n====================================================\n"

    echo -e "${BOLD}${BLUE}Updating system ......${NC}"
    apt-get update
    apt-get upgrade -y

    echo -e "${BOLD}${BLUE}Installing tools ......${NC}"
    apt-get install -y htop iftop neovim git curl wget

    echo -e "${BOLD}${BLUE}Installing IBMTPM dependencies ......${NC}"
    apt-get install -y build-essential make gcc libssl-dev
    # tss dependencies on ubuntu packages
    # apt-get install -y libtss0 libtss-dev libtss2-dev libtss2-doc libtss2-esys0 libtss2-esys-3.0.2-0 libtss2-fapi1 libtss2-mu0 libtss2-policy0 libtss2-rc0 libtss2-sys1 libtss2-tcti-cmd0 libtss2-tcti-device0 libtss2-tcti-libtpms0 libtss2-tcti-mssim0 libtss2-tcti-pcap0 libtss2-tcti-spi-helper0 libtss2-tcti-swtpm0 libtss2-tcti-tabrmd-dev libtss2-tcti-tabrmd0 libtss2-tctidr0

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
}

install_ibmtpmtss () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMTSS${NC}\n====================================================\n"

    echo -e "${BOLD}${BLUE}Cleaning up ......${NC}"
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmtss}/utils/"
        make -f makefiletpm20 clean
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmtss}/utils/"
        make -f makefiletpmc clean
        cd "${path_ibmtss}/utils12/"
        make -f makefiletpmc clean
    else 
        echo -e "${BOLD}${RED}Invalid verMode${NC}"
        exit 1
    fi

    echo -e "${BOLD}${BLUE}Compiling IBMTSS ......${NC}"
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmtss}/utils/"
        make -f makefiletpm20
    elif [ $verMode == 2 ]; then
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
}

install_ibmswtpm () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMTPM${NC}\n====================================================\n"

    echo -e "${BOLD}${BLUE}Cleaning up ......${NC}"
    cd "${path_ibmtpm}/src/"
    make clean

    echo -e "${BOLD}${BLUE}Compiling IBMTPM ......${NC}"
    cd "${path_ibmtpm}/src/"
    make

    echo -e "${BOLD}${BLUE}Creating symbolic link to ${path_ibmtpm} ......${NC}"
    ln -s "${path_ibmtpm}" "${base_dir}/ibmtpm"

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMTPM Complete${NC}\n====================================================\n"
}

install_ibmacs () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMACS${NC}\n====================================================\n"

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

    echo -e "${BOLD}${BLUE}Setting include path ......${NC}"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"
    export PATH="${PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"

    echo -e "${BOLD}${BLUE}Creating symbolic link to ${path_ibmacs} ......${NC}"
    ln -s "${path_ibmacs}/acs" "${base_dir}/ibmacs"

    echo -e "${BOLD}${BLUE}Setting html directory ......${NC}"
    mkdir ${html_dir}
    chown root ${html_dir}
    chgrp root ${html_dir}
    chmod 777 ${html_dir}

    echo -e "${BOLD}${BLUE}Creating symbolic link to ${c_src_dir} ......${NC}"
    ln -s "${c_src_dir}" "${c_link_dir}"

    echo -e "${BOLD}${BLUE}Compiling IBMACS and setting include path ......${NC}"
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils"
        export LIBRARY_PATH="${path_ibmtss}/utils"
        make clean
        make
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        export LIBRARY_PATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        if [ $acsMode == 1 ]; then
            # for Server
            make -f makefiletpm12 clean
            make -f makefiletpm12
        elif [ $acsMode == 2 ]; then
            # for Client
            make -f makefiletpmc clean
            make -f makefiletpmc
        else 
            echo -e "${BOLD}${RED}Invalid acsMode${NC}"
            exit 1
        fi
    else 
        echo -e "${BOLD}${RED}Invalid verMode${NC}"
        exit 1
    fi

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing IBMACS Complete${NC}\n====================================================\n"
}
