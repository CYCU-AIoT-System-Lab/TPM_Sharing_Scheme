#!/bin/bash

# ==================================================================================================
# Parameters
# Param - path
download_dir="/home/user/Downloads"      # default: /home/user/Downloads
base_dir="/opt"                          # default: /opt
html_dir="/var/www/html/acs"             # default: /var/www/html/acs
c_json_lib_dir="/usr/include/json-c"     # default: /usr/include/json-c
c_json_lib_link_dir="/usr/include/json"  # default: /usr/include/json
nvim_dir="/home/user/.config/nvim"       # default: /home/user/.config/nvim
bashrc_dir="/home/user/.bashrc"          # default: /home/user/.bashrc
# Param - url
nvim_config_url="https://raw.githubusercontent.com/belongtothenight/config-files/main/ubuntu_init.vim"
# Param - version
ibmtss_ver="2.1.1"                       # latest: 2.1.1
ibmtpm_ver="1682"                        # latest: 1682
ibmacs_ver="1658"                        # latest: 1658
# Param - mode
verMode=2                                # 1: TPM 2.0, 2: TPM 1.2 & 2.0
TPMMode=2                                # 1: Physical TPM, 2: Software TPM
acsMode=1                                # 1: Server, 2: Client
# Param - job
install_req=1                            # 0: No, 1: Yes
config_nvim=1                            # 0: No, 1: Yes (buggy)
setup_ibmtpmtss_env=0                    # 0: No, 1: Yes
compile_ibmtpmtss=0                      # 0: No, 1: Yes
setup_ibmswtpm_env=0                     # 0: No, 1: Yes
compile_ibmswtpm=0                       # 0: No, 1: Yes
setup_ibmacs_env=0                       # 0: No, 1: Yes
compile_ibmacs=0                         # 0: No, 1: Yes
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

# Check if running as root
if [[ $(/usr/bin/id -u) -ne 0 ]]; then
    echo "${BOLD}${RED}Please run with command: sudo bash setup.sh${NC}"
    exit 1
fi


echo -e "\n====================================================\n>>${BOLD}${GREEN}Setup${NC}\n====================================================\n"

# Install requirements for development, building, and testing
# Download ibmtss, ibmtpm, and ibmacs from sourceforge
# Extract ibmtss, ibmtpm, and ibmacs
# Only need to setup once (can re-run)
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

    echo -e "${BOLD}${BLUE}Creating directories ......${NC}"
    mkdir "${path_ibmtss}"
    mkdir "${path_ibmtpm}"
    mkdir "${path_ibmacs}"

    echo -e "${BOLD}${BLUE}Downloading IBMTPM ......${NC}"
    wget "https://sourceforge.net/projects/ibmtpm20tss/files/${fn_ibmtss}/download" -O "${path_ibmtss}/${fn_ibmtss}"
    wget "https://sourceforge.net/projects/ibmswtpm2/files/${fn_ibmtpm}/download" -O "${path_ibmtpm}/${fn_ibmtpm}"
    wget "https://sourceforge.net/projects/ibmtpm20acs/files/${fn_ibmacs}/download" -O "${path_ibmacs}/${fn_ibmacs}"

    echo -e "${BOLD}${BLUE}Extracting files ......${NC}"
    tar -zxvf "${path_ibmtss}/${fn_ibmtss}" -C ${path_ibmtss}
    tar -zxvf "${path_ibmtpm}/${fn_ibmtpm}" -C ${path_ibmtpm}
    tar -zxvf "${path_ibmacs}/${fn_ibmacs}" -C ${path_ibmacs}

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Installing requirements Complete${NC}\n====================================================\n"
}

# Configure neovim
# Only need to setup once (can re-run)
config_nvim () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Configuring neovim${NC}\n====================================================\n"

    echo -e "${BOLD}${BLUE}Configuring neovim ......${NC}"
    mkdir "${nvim_dir}"
    wget "${nvim_config_url}" -O "${nvim_dir}/init.nvim"

    # echo -e "${BOLD}${BLUE}Installing vim plug ......${NC}"
    # sh -c 'curl -fLo "${XDG_DATA_HOME:-$HOME/.local/share}"/nvim/site/autoload/plug.vim --create-dirs https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim'

    # echo -e "${BOLD}${BLUE}Installing nodejs ......${NC}"
    # apt-get install -y nodejs-dev node-gyp libssl1.0-dev
    # apt-get install -y nodejs npm
    # source ${bashrc_dir}
    # npm cache clean -f
    # npm install -g n
    # n stable
    source ${bashrc_dir}

    # Commands to install plugins and coc extensions
    # :PlugInstall
    # :

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Configuring neovim Complete${NC}\n====================================================\n"
}

# Set environment variables for ibmtss, and create symbolic link to ibmtss
# Only need to setup once (can re-run)
setup_ibmtpmtss_env () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Setting up IBMTSS Environment${NC}\n====================================================\n"

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

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Setting up IBMTSS Environment Complete${NC}\n====================================================\n"
}

# Compile ibmtss
# Can be run multiple times for code adjustment
compile_ibmtpmtss () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Compiling IBMTSS${NC}\n====================================================\n"

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

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Compiling IBMTSS Complete${NC}\n====================================================\n"
}

# Create symbolic link to ibmswtpm
# Only need to setup once (can re-run)
setup_ibmswtpm_env () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Setting up IBMSWTPM Environment${NC}\n====================================================\n"

    echo -e "${BOLD}${BLUE}Creating symbolic link to ${path_ibmtpm} ......${NC}"
    ln -s "${path_ibmtpm}" "${base_dir}/ibmtpm"

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Setting up IBMSWTPM Environment Complete${NC}\n====================================================\n"
}

# Compile ibmswtpm
# Can be run multiple times for code adjustment
compile_ibmswtpm () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Compiling IBMTPM${NC}\n====================================================\n"

    echo -e "${BOLD}${BLUE}Cleaning up ......${NC}"
    cd "${path_ibmtpm}/src/"
    make clean

    echo -e "${BOLD}${BLUE}Compiling IBMTPM ......${NC}"
    cd "${path_ibmtpm}/src/"
    make

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Compiling IBMTPM Complete${NC}\n====================================================\n"
}

# Install requirements for ibmacs, create mysql database, set environment variables, link directories, and generate directory for webpage
# Only need to setup once (can re-run)
setup_ibmacs_env () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Setting Up IBMACS Environment${NC}\n====================================================\n"

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

    echo -e "${BOLD}${BLUE}Creating symbolic link to ${c_json_lib_dir} ......${NC}"
    ln -s "${c_json_lib_dir}" "${c_json_lib_link_dir}"

    echo -e "${BOLD}${BLUE}Setting include path ......${NC}"
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils"
        export LIBRARY_PATH="${path_ibmtss}/utils"
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        export LIBRARY_PATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
    else 
        echo -e "${BOLD}${RED}Invalid verMode${NC}"
        exit 1
    fi

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Setting Up IBMACS Environment Complete${NC}\n====================================================\n"
}

# Compile ibmacs
# Can be run multiple times for code adjustment
compile_ibmacs () {
    echo -e "\n====================================================\n>>${BOLD}${GREEN}Compiling IBMACS${NC}\n====================================================\n"

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

    echo -e "\n====================================================\n>>${BOLD}${GREEN}Compiling IBMACS Complete${NC}\n====================================================\n"
}

if [ $install_req == 1 ]; then install_req; fi

if [ $config_nvim == 1 ]; then config_nvim; fi

if [ $setup_ibmtpmtss_env == 1 ]; then setup_ibmtpmtss_env; fi
if [ $compile_ibmtpmtss == 1 ]; then compile_ibmtpmtss; fi

if [ $setup_ibmswtpm_env == 1 ]; then setup_ibmswtpm_env; fi
if [ $compile_ibmswtpm == 1 ]; then compile_ibmswtpm; fi

if [ $setup_ibmacs_env == 1 ]; then setup_ibmacs_env; fi
if [ $compile_ibmacs == 1 ]; then compile_ibmacs; fi

echo -e "\n====================================================\n>>${BOLD}${GREEN}Setup Complete${NC}\n====================================================\n"
