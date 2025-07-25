#!/bin/bash

source "../common/functions.sh"
source "./function_ibmtpm.sh"

dirname="setup_ibmtpm"
filename="setup_sudo"
current_dir=$(pwd)

# Install requirements for development, building, and testing
# Download ibmtss, ibmtpm, and ibmacs from sourceforge
# Extract ibmtss, ibmtpm, and ibmacs
# Only need to setup once (can re-run)
install_req () {
    # Download CA from Infineon website
    # Argument 1: CA number
    download_optiga_CA () {
        echo_notice "${dirname}" "${filename}-download_optiga_CA" "Retrieving Infineon TPM9670 CA $1 ..."
        wget $wget_gflag "https://pki.infineon.com/OptigaRsaMfrCA$1/OptigaRsaMfrCA$1.crt" -P "${tss_cert_rootcert_dir}"
        wget $wget_gflag "https://pki.infineon.com/OptigaEccMfrCA$1/OptigaEccMfrCA$1.crt" -P "${tss_cert_rootcert_dir}"

        echo_notice "${dirname}" "${filename}-download_optiga_CA" "Converting CA $1 from CRT to PEM ..."
        openssl x509 -in "${tss_cert_rootcert_dir}/OptigaRsaMfrCA$1.crt" -inform der -outform pem -out "${tss_cert_rootcert_dir}/OptigaRsaMfrCA$1.pem"
        openssl x509 -in "${tss_cert_rootcert_dir}/OptigaEccMfrCA$1.crt" -inform der -outform pem -out "${tss_cert_rootcert_dir}/OptigaEccMfrCA$1.pem"

        echo_notice "${dirname}" "${filename}-download_optiga_CA" "Removing CA $1 CRT ..."
        rm "${tss_cert_rootcert_dir}/OptigaRsaMfrCA$1.crt"
        rm "${tss_cert_rootcert_dir}/OptigaEccMfrCA$1.crt"
    }

    echo_notice "${dirname}" "${filename}-install_req" "Creating directories ..."
    err_conti_exec "mkdir ${path_ibmtss}" "${dirname}" "setup-install_req"
    err_conti_exec "mkdir ${path_ibmtpm}" "${dirname}" "setup-install_req"
    err_conti_exec "mkdir ${path_ibmacs}" "${dirname}" "setup-install_req"

    echo_notice "${dirname}" "${filename}-install_req" "Downloading IBMTPMTSS ..."
    err_retry_exec "wget $wget_gflag https://sourceforge.net/projects/ibmtpm20tss/files/${fn_ibmtss}/download -O ${path_ibmtss}/${fn_ibmtss}" 1 5 "${dirname}" "setup-install_req"
    echo_notice "${dirname}" "${filename}-install_req" "Downloading IBMSWTPM ..."
    err_retry_exec "wget $wget_gflag https://sourceforge.net/projects/ibmswtpm2/files/${fn_ibmtpm}/download -O ${path_ibmtpm}/${fn_ibmtpm}" 1 5 "${dirname}" "setup-install_req"
    echo_notice "${dirname}" "${filename}-install_req" "Downloading IBMACS ..."
    err_retry_exec "wget $wget_gflag https://sourceforge.net/projects/ibmtpm20acs/files/${fn_ibmacs}/download -O ${path_ibmacs}/${fn_ibmacs}" 1 5 "${dirname}" "setup-install_req"

    echo_notice "${dirname}" "${filename}-install_req" "Extracting IBMTPMTSS ..."
    tar $tar_gflag "${path_ibmtss}/${fn_ibmtss}" -C ${path_ibmtss}
    echo_notice "${dirname}" "${filename}-install_req" "Extracting IBMSWTPM ..."
    tar $tar_gflag "${path_ibmtpm}/${fn_ibmtpm}" -C ${path_ibmtpm}
    echo_notice "${dirname}" "${filename}-install_req" "Extracting IBMACS ..."
    tar $tar_gflag "${path_ibmacs}/${fn_ibmacs}" -C ${path_ibmacs}

    echo_notice "${dirname}" "${filename}-setup_ibmtpmtss_env" "Creating symbolic link to ${path_ibmtss} ..."
    err_conti_exec "ln -s ${path_ibmtss} ${base_dir}/ibmtss" "${dirname}" "setup-setup_ibmtpmtss_env"
    echo_notice "${dirname}" "${filename}-setup_ibmswtpm_env" "Creating symbolic link to ${path_ibmtpm} ..."
    err_conti_exec "ln -s ${path_ibmtpm} ${base_dir}/ibmtpm" "${dirname}" "setup-setup_ibmswtpm_env"
    echo_notice "${dirname}" "${filename}-setup_ibmacs_env" "Creating symbolic link to ${path_ibmacs} ..."
    err_conti_exec "ln -s ${path_ibmacs}/acs ${base_dir}/ibmacs" "${dirname}" "setup-setup_ibmacs_env"
    echo_notice "${dirname}" "${filename}-setup_ibmacs_env" "Creating symbolic link to ${c_json_lib_dir} ..."
    err_conti_exec "ln -s ${c_json_lib_dir} ${c_json_lib_link_dir}" "${dirname}" "setup-setup_ibmacs_env"

    echo_notice "${dirname}" "${filename}-setup_install_req" "Installing IBMACS dependencies ..."
    if [ $acsMode == 1 ]; then
        # for Server
        if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ] || [ $install_platform -eq 5 ]; then
            aptins "libjson-c-dev"
            aptins "apache2"
            aptins "php"
            aptins "php-dev"
            aptins "php-mysql"
            aptins "mysql-server"
            aptins "libmysqlclient-dev"
        elif [ $install_platform -eq 2 ] || [ $install_platform -eq 3 ]; then
            aptins "libjson-c-dev"
            aptins "apache2"
            aptins "php"
            aptins "php-dev"
            aptins "php-mysql"
            aptins "mariadb-server"
            aptins "libmariadb-dev-compat"
        else
            echo_error "${dirname}" "setup-setup_ibmacs_env" "Invalid install_platform" 1
        fi
    elif [ $acsMode == 2 ]; then
        # for Client
        aptins "libjson-c-dev"
        # following dependencies are not required officially, but required for this script
        if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ] || [ $install_platform -eq 5 ]; then
            aptins "mysql-server"
            aptins "libmysqlclient-dev"
        elif [ $install_platform -eq 2 ] || [ $install_platform -eq 3 ]; then
            aptins "mariadb-server"
            aptins "libmariadb-dev-compat"
        else
            echo_error "${dirname}" "setup-setup_ibmacs_env" "Invalid install_platform" 1
        fi
    else 
        echo_warn "${dirname}" "${filename}-setup_ibmacs_env" "Invalid acsMode"
        exit 1
    fi

    # Brute force all available CA from Infineon website
    #for i in $(seq -f "%03g" 0 070); do
    #    err_conti_exec "download_optiga_CA $i" & # parallel execution
    #done
    #wait
    download_optiga_CA 042
}

# Set environment variables for ibmtss, and create symbolic link to ibmtss
# Can re-run to update environment variables to switch between physical TPM and software TPM
# Only need to setup once (can re-run)
setup_ibmtpmtss_env () {
    echo_notice "${dirname}" "${filename}-setup_ibmtpmtss_env" "Setting path env variable ..."
    if [ $TPMMode == 1 ]; then
        # for Pysical TPM
        export TPM_INTERFACE_TYPE=dev
    elif [ $TPMMode == 2 ]; then
        # for Software TPM
        export TPM_INTERFACE_TYPE=socsim
    else 
        echo -e "${BOLD}${RED}Invalid TPMMode${NC}"
        echo_warn "${dirname}" "${filename}-setup_ibmtpmtss_env" "Invalid TPMMode"
        exit 1
    fi
    export TPM_COMMAND_PORT="${tpm_command_port}"

    echo_notice "${dirname}" "${filename}-setup_ibmtpmtss_env" "Replacing path in ${tss_cert_rootcert_dir}/rootcerts.txt ..."
    cp "${tss_cert_rootcert_dir}/rootcerts.txt" "${tss_cert_rootcert_dir}/rootcerts.txt.bak"
    ls ${tss_cert_rootcert_dir} -I *.txt* > "${tss_cert_rootcert_dir}/rootcerts.txt"
    sed -i -e "s_.*_${tss_cert_rootcert_dir}/&_" "${tss_cert_rootcert_dir}/rootcerts.txt"
}

# Compile ibmtss
# Can be run multiple times for code adjustment
compile_ibmtpmtss () {
    echo_notice "${dirname}" "${filename}-compile_ibmtpmtss" "Cleaning up with make ..."
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmtss}/utils/"
        make $make_gflag -f makefiletpm20 clean
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmtss}/utils/"
        make $make_gflag -f makefiletpmc clean
        cd "${path_ibmtss}/utils12/"
        make $make_gflag -f makefiletpmc clean
    else 
        echo_warn "${dirname}" "${filename}-compile_ibmtpmtss" "Invalid verMode"
        exit 1
    fi

    echo_notice "${dirname}" "${filename}-compile_ibmtpmtss" "Compiling IBMTSS ..."
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmtss}/utils/"
        make $make_gflag -f makefiletpm20 -j$(nproc)
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmtss}/utils/"
        make $make_gflag -f makefiletpmc -j$(nproc)
        cd "${path_ibmtss}/utils12/"
        make $make_gflag -f makefiletpmc -j$(nproc)
    else 
        echo_warn "${dirname}" "${filename}-compile_ibmtpmtss" "Invalid verMode"
        exit 1
    fi
}
export LD_LIBRARY_PATH="${path_ibmtss}/utils:$LD_LIBRARY_PATH"

# Create symbolic link to ibmswtpm
# Only need to setup once (can re-run)
setup_ibmswtpm_env () {
    if ! [ $install_platform -eq 3 ]; then
        echo_notice "${dirname}" "${filename}-setup_ibmswtpm_env" "Export environment variable for SWTPM socket interface ..."
        echo "export TPM_COMMAND_PORT=$tpm_command_port TPM_PLATFORM_PORT=$tpm_socket_port TPM_SERVER_NAME=$acs_demo_server_ip TPM_INTERFACE_TYPE=socsim TPM_SERVER_TYPE=raw" >> ~/.bashrc
        echo "export TPM2TOOLS_TCTI=\"swtpm:port=$tpm_command_port\"" >> ~/.bashrc
    fi
}

# Compile ibmswtpm
# Can be run multiple times for code adjustment
compile_ibmswtpm () {
    echo_notice "${dirname}" "${filename}-compile_ibmswtpm" "Cleaning up with make ..."
    cd "${path_ibmtpm}/src/"
    make $make_gflag clean

    echo_notice "${dirname}" "${filename}-compile_ibmswtpm" "Compiling IBMTPM ..."
    cd "${path_ibmtpm}/src/"
    make $make_gflag -j$(nproc)
}

# Install requirements for ibmacs, create mysql database, set environment variables, link directories, and generate directory for webpage
# Only need to setup once (can re-run)
setup_ibmacs_env () {
    # ACS source platform adaption
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ] || [ $install_platform -eq 5 ]; then
        :
    elif [ $install_platform -eq 2 ] || [ $install_platform -eq 3 ]; then
        echo_notice "${dirname}" "${filename}-setup_ibmacs_env" "Adding stdbool.h to IBMACS/acs/commonjson.c"
        sed -i '39 i #include <stdbool.h>' "${path_ibmacs}/acs/commonjson.c"

        echo_notice "${dirname}" "${filename}-setup_ibmacs_env" "Replacing \"FALSE\" with \"false\" in IBMACS/acs/commonjson.c"
        sed -i 's/FALSE/false/g' "${path_ibmacs}/acs/commonjson.c"

        #echo_notice "${dirname}" "${filename}-setup_ibmacs_env" "Replacing all mysql/mysql.h with mariadb/mysql.h in all files"
        #for file in $(grep -rl "mysql/mysql.h" "${path_ibmacs}/acs/"); do
        #    sed -i 's/mysql\/mysql.h/mariadb\/mysql.h/g' $file
        #done
    else
        echo_error "${dirname}" "setup-setup_ibmacs_env" "Invalid install_platform" 1
    fi

    if [ $acsMode == 1 ]; then
        # https://stackoverflow.com/questions/24270733/automate-mysql-secure-installation-with-echo-command-vvia-a-shell-script
        echo_notice "${dirname}" "${filename}-setup_ibmacs_env" "Setting database ..."
        cd "${path_ibmacs}/acs/"
        mysql -Bse "CREATE DATABASE IF NOT EXISTS ${mysql_database};"
        mysql -Bse "CREATE USER IF NOT EXISTS '${mysql_user}'@'${acs_demo_server_ip}' IDENTIFIED BY '${mysql_password}';"
        mysql -Bse "GRANT ALL ON ${mysql_database}.* TO '${mysql_user}'@'${acs_demo_server_ip}';"
        mysql -D ${mysql_database} < "${path_ibmacs}/acs/dbinit.sql"
    fi

    echo_notice "${dirname}" "${filename}-setup_ibmacs_env" "Setting include path ..."
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"
    export PATH="${PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"

    echo_notice "${dirname}" "${filename}-setup_ibmacs_env" "Setting html directory ..."
    err_conti_exec "mkdir -p ${html_dir}" "${dirname}" "setup-setup_ibmacs_env"
    chown root ${html_dir}
    chgrp root ${html_dir}
    chmod 777 ${html_dir}

    echo_notice "${dirname}" "${filename}-setup_ibmacs_env" "Setting include path ..."
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
        echo_warn "${dirname}" "${filename}-setup_ibmacs_env" "Invalid verMode"
        exit 1
    fi
}

# Compile ibmacs
# Can be run multiple times for code adjustment
compile_ibmacs () {
    echo_notice "${dirname}" "${filename}-compile_ibmacs" "Compiling IBMACS and setting include path ..."
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils"
        export LIBRARY_PATH="${path_ibmtss}/utils"
        make $make_gflag clean
        make $make_gflag -j$(nproc)
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        export LIBRARY_PATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        if [ $acsMode == 1 ]; then
            # for Server
            make $make_gflag -f makefiletpmc clean
            make $make_gflag -f makefiletpmc -j$(nproc)
        elif [ $acsMode == 2 ]; then
            # for Client
            make $make_gflag -f makefiletpm12 clean
            make $make_gflag -f makefiletpm12 -j$(nproc)
            make $make_gflag -f makefiletpmc clean
            make $make_gflag -f makefiletpmc -j$(nproc)
        else 
            echo_warn "${dirname}" "${filename}-compile_ibmacs" "Invalid acsMode"
            exit 1
        fi
    else 
        echo_warn "${dirname}" "${filename}-compile_ibmacs" "Invalid verMode"
        exit 1
    fi
}

# Open demo webpage with firefox
# Can be run multiple times
open_demo_webpage () {
    echo_notice "${dirname}" "${filename}-open_demo_webpage" "Restarting apache2 to reload all functionalities ..."
    sudo systemctl restart apache2.service
    echo_notice "${dirname}" "${filename}-open_demo_webpage" "Opening demo webpage with new terminal ..."
    echo_notice "${dirname}" "${filename}-open_demo_webpage" "If website is displaying ${YELLOW}${BOLD}fatal error${END} or ${YELLOW}${BOLD}mysqli related error${END}, please execute ${YELLOW}${BOLD}sudo systemctl restart apache2.service${END}!"
    lc1="source ${current_dir}/../common/functions.sh"
    lc2="echo_notice \"setup_ibmtpm\" \"setup-open_demo_webpage\" \"Opening demo webpage with new terminal ...\""
    lc3="echo_notice \"setup_ibmtpm\" \"setup-open_demo_webpage\" \"If website is displaying ${YELLOW}${BOLD}fatal error${END} or ${YELLOW}${BOLD}mysqli related error${END}, please execute ${YELLOW}${BOLD}sudo systemctl restart apache2.service${END}!\""
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ] || [ $install_platform -eq 5 ]; then
        lc4="sudo -u $user bash -c \"firefox --new-tab -url ${acs_demo_url_A} --new-tab -url ${repo_url} &\""
        newGterm "FireFox Browser" "$bash_gflag" "$lc1; $lc2; $lc3; $lc4" 1
    else
        lc4="sudo -u $user bash -c \"chromium-browser --new-window ${acs_demo_url_B} ${repo_url} &\""
        newLXterm "Chromium Browser" "$lc1; $lc2; $lc3; $lc4" 1
    fi
}

# Generate CA certificate and key
# Only need to setup once (can re-run)
generate_CA () {
    echo_warn "${dirname}" "${filename}-generate_CA" "Function not implemented"
    echo_warn "${dirname}" "${filename}-generate_CA" "Refer to ${sym_link_ibmacs}/README.txt line 171 for steps."
    echo_notice "${dirname}" "${filename}-generate_CA" "Generated CAs in ${sym_link_ibmtss}/utils ......"
    ls "${sym_link_ibmtss}/utils/"*.pem
    echo_notice "${dirname}" "${filename}-generate_CA" "Generated CAs in ${sym_link_ibmacs} ......"
    ls "${sym_link_ibmacs}/"*.pem
}

# Activate TPM Server in new terminal
# Only need to setup once (can re-run)
activate_TPM_server () {
    TPM_server_executed=1
    # apply TPMMode for ibmtss
    setup_ibmtpmtss_env

    echo_notice "${dirname}" "${filename}-activate_TPM_server" "Starting TPM simulator (server) on new temrinal ..."
    cd "${sym_link_ibmtpm}/src/"
    lc1="source ${current_dir}/../common/functions.sh"
    if [ $new_swtpm -ne 1 ]; then
        lc2="echo_notice \"setup_ibmtpm\" \"setup-activate_TPM_server\" \"Starting legacy TPM simulator (server) on new temrinal ...\n\""
        lc3="./tpm_server"
    else
        lc2="echo_notice \"setup_ibmtpm\" \"setup-activate_TPM_server\" \"Starting TPM simulator (server) on new temrinal ...\n\""
        # ref: https://github.com/stefanberger/swtpm/wiki/Using-the-IBM-TSS-with-swtpm
        lc3="mkdir $swtpm_socket_device || :; swtpm socket --tpmstate dir=$swtpm_socket_device --tpm2 --ctrl type=tcp,port=$tpm_socket_port --server type=tcp,port=$tpm_command_port --flags not-need-init"
    fi
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ] || [ $install_platform -eq 5 ]; then
        newGterm "TPM SERVER" "$bash_gflag" "$lc1; $lc2; $lc3" 1
    else
        newLXterm "TPM SERVER" "$lc1; $lc2; $lc3" 1
    fi
}

# Send TPM activation command to TPM server
# Only need to setup once (can re-run)
activate_TPM_client () {
    TPM_client_executed=1
    echo_notice "${dirname}" "${filename}-activate_TPM_client" "Sending activation command to TPM on new terminal ..."
    cd "${sym_link_ibmtss}/utils/"
    lc1="source ${current_dir}/../common/functions.sh"
    lc2="echo_notice \"setup_ibmtpm\" \"setup-activate_TPM_client\" \"Sending activation command to TPM on new terminal ...\n\""
    if [ $new_swtpm -ne 1 ]; then
        # legacy SWTPM will hang here (it's fine)
        lc3="./powerup"
        lc4="./startup"
    else
        # updated SWTPM will just pass through and exit
        lc3="export TPM2TOOLS_TCTI=\"swtpm:port=$tpm_command_port\" LD_LIBRARY_PATH=${path_ibmtss}/utils:$LD_LIBRARY_PATH"
        lc4="tpm2_startup -c; sleep 5; tpm2_startup -c"
    fi
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ] || [ $install_platform -eq 5 ]; then
        #newGterm "TPM ACT CMD" "-x" "$lc1; $lc2; $lc3; $lc4" 1
        newGterm "TPM ACT CMD" "" "$lc1; $lc2; $lc3; $lc4" 1
    else
        newLXterm "TPM ACT CMD" "$lc1; $lc2; $lc3; $lc4" 1
    fi
}

# Create EK certificate and key, activated TPM on new terminal
# Only need to setup once (can re-run)
# Only for SWTPM/vTPM
generate_EK () {
    if [ $TPMMode == 1 ]; then
        echo_notice "${dirname}" "${filename}-generate_EK" "TPMMode is 1, skip generating EK"
    elif [ $TPMMode == 2 ]; then
        activate_TPM_server
        activate_TPM_client

        echo_notice "${dirname}" "${filename}-generate_EK" "Backing up NVChip ......"
        err_conti_exec "cp ${path_NV} ${path_NV}.bak" "${dirname}" "setup-generate_EK"

        cd "${sym_link_ibmtss}/utils/"
        echo_notice "${dirname}" "${filename}-generate_EK" "Generating RSAEK and load into NV ......"
        ./createekcert -rsa 2048 -cakey cakey.pem -capwd rrrr -vv

        echo_notice "${dirname}" "${filename}-generate_EK" "Generating ECCEK and load into NV ......"
        ./createekcert -ecc nistp256 -cakey cakeyecc.pem -capwd rrrr -caalg ec -vv
    else
        echo_warn "${dirname}" "${filename}-generate_EK" "Invalid TPMMode"
        exit 1
    fi
}

# Retrieve NVChip
# Only need to setup once (can re-run)
retrieve_EK () {
    echo_notice "${dirname}" "${filename}-retrieve_EK" "Retrieve EK from NVChip ......"
    cd "${sym_link_ibmtss}/utils/"
    ./nvread -ha 01c00002 | awk 'NR==1 {next} {print}' | xxd -r -ps | base64 | sed -e '1i -----BEGIN CERTIFICATE-----' -e '$a -----END CERTIFICATE-----' > VMW_EK_CACERT.pem
    ./nvread -ha 01c0000a | awk 'NR==1 {next} {print}' | xxd -r -ps | base64 | sed -e '1i -----BEGIN CERTIFICATE-----' -e '$a -----END CERTIFICATE-----' > VMW_EKECC_CACERT.pem
}

# Set ACS MYSQL setting
# Only need to setup once (can re-run)
set_acs_sql_setting () {
    echo_notice "${dirname}" "${filename}-set_acs_sql_setting" "Setting ACS MYSQL Setting ..."
    export ACS_SQL_USERID="${mysql_user}"
    export ACS_SQL_PASSWORD="${mysql_password}"

    if [ $force_acs_sql_setting == 1 ]; then
        echo_warn "${dirname}" "${filename}-set_acs_sql_setting" "Forcing ACS MySQL Setting ..."
        cp "${html_dir}/dbconnect.php" "${html_dir}/dbconnect.php.bak"
        sed -i 's@($acs_sql_host, $acs_sql_userid, $acs_sql_password, $acs_sql_database)@'"(\"${acs_demo_server_ip}\", \"${mysql_user}\", \"${mysql_password}\", \"${mysql_database}\")"'@' "${html_dir}/dbconnect.php"
    else
        echo_warn "${dirname}" "${filename}-set_acs_sql_setting" "Not Forcing ACS MySQL Setting ..."
    fi
}

# Active ACS Demo Server
# Can be run multiple times
active_ACS_Demo_Server () {
    if [ $SCmachineMode == 1 ]; then
        echo_notice "${dirname}" "${filename}-active_ACS_Demo_Server" "Activating ACS Demo on same machine ..."
        err_conti_exec "mkdir ${tpm_data_dir}" "${dirname}" "setup-active_ACS_Demo_Server"
        export TPM_DATA_DIR="${tpm_data_dir}"
    elif [ $SCmachineMode == 2 ]; then
        echo_notice "${dirname}" "${filename}-active_ACS_Demo_Server" "Activating ACS Demo on different machine ..."
    else 
        echo_warn "${dirname}" "${filename}-active_ACS_Demo_Server" "Invalid SCmachineMode"
        exit 1
    fi

    set_acs_sql_setting

    echo_notice "${dirname}" "${filename}-active_ACS_Demo_Server" "Activating ACS Demo on new terminal ..."
    cd ${path_ibmacs}/acs
    lc1="source ${current_dir}/../common/functions.sh"
    lc2="echo_notice \"setup_ibmtpm\" \"setup-active_ACS_Demo_Server\" \"Activating ACS Demo ...\""
    lc3="export ACS_PORT=${acs_port} LD_LIBRARY_PATH=${path_ibmtss}/utils:$LD_LIBRARY_PATH"
    lc4="log_date_time \"./server -vv -root ${tss_cert_rootcert_dir}/rootcerts.txt -imacert imakey.der\" \"$log4j_time_format\" \"${acs_demo_server_log_dir}\" \"default\""
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ] || [ $install_platform -eq 5 ]; then
        newGterm "ACS SERVER" "$bash_gflag" "$lc1; $lc2; $lc3; $lc4" 1
    else
        newLXterm "ACS SERVER" "$lc1; $lc2; $lc3; $lc4" 1
    fi
}

# Active ACS Demo Client
# Can be run multiple times
active_ACS_Demo_Client () {
    export LD_LIBRARY_PATH="${path_ibmtss}/utils:$LD_LIBRARY_PATH"
    cd "${path_ibmacs}/acs"
    if [ $SCmachineMode == 1 ]; then
        echo_notice "${dirname}" "${filename}-active_ACS_Demo_Client" "Activating ACS Demo on local machine ..."
        log_date_time "./clientenroll -alg rsa -vv -ho ${acs_demo_server_ip} -co akcert.pem" "$log4j_time_format" "${acs_demo_client_log_dir}" "default"
    elif [ $SCmachineMode == 2 ]; then
        echo_notice "${dirname}" "${filename}-active_ACS_Demo_Client" "Activating ACS Demo on remote machine ..."
        log_date_time "./clientenroll -alg ec -vv -ho ${acs_demo_server_ip} -ma ${acs_demo_client_ip} -co akeccert.pem" "$log4j_time_format" "${acs_demo_client_log_dir}" "default"
    else 
        echo_warn "${dirname}" "${filename}-active_ACS_Demo_Client" "Invalid SCmachineMode"
        exit 1
    fi
}

# Active ACS Demo verify
# Can be run multiple times
active_ACS_Demo_verify () {
    export LD_LIBRARY_PATH="${path_ibmtss}/utils:$LD_LIBRARY_PATH"
    cd "${sym_link_ibmacs}"
    echo_notice "${dirname}" "${filename}-active_ACS_Demo_verify" "Extending TPM2BIOS.LOG ..."
    log_date_time "${sym_link_ibmtss}/utils/eventextend -if ${swtpm_bios_log_dir} -tpm -v" "$log4j_time_format" "${acs_demo_verify_tpm2bios_log_dir}" "default"

    echo_notice "${dirname}" "${filename}-active_ACS_Demo_verify" "Extending IMASIG.LOG (it can take a few minutes) ..."
    log_date_time "${sym_link_ibmtss}/utils/imaextend -if ${ima_sig_log_dir} -le -v" "$log4j_time_format" "${acs_demo_verify_imasig_log_dir}" "default"

    if [ $SCmachineMode == 1 ]; then
        # for Local
        log_date_time "${sym_link_ibmacs}/client -alg rsa -ifb ${swtpm_bios_log_dir} -ifi ${ima_sig_log_dir} -ho ${acs_demo_server_ip} -vv" "$log4j_time_format" "${acs_demo_verify_client_log_dir}" "default"
    elif [ $SCmachineMode == 2 ]; then
        # for Remote
        if ! [ -z ${interval+x} ]; then
            # if interval is set, perform forever attestation
            attest_cnt=0
            while true; do
                log_date_time "${sym_link_ibmacs}/client -alg ec -ifb ${swtpm_bios_log_dir} -ifi ${ima_sig_log_dir} -ho ${acs_demo_server_ip} -vv -ma ${acs_demo_client_ip}" "$log4j_time_format" "${acs_demo_verify_client_log_dir}" "default"
                attest_cnt=$((attest_cnt + 1))
                echo "wait $interval >> performed attestation: $attest_cnt"
                sleep $interval
            done
        else
            # if interval is not set, perform just once
            log_date_time "${sym_link_ibmacs}/client -alg ec -ifb ${swtpm_bios_log_dir} -ifi ${ima_sig_log_dir} -ho ${acs_demo_server_ip} -vv -ma ${acs_demo_client_ip}" "$log4j_time_format" "${acs_demo_verify_client_log_dir}" "default"
        fi
    else 
        echo_warn "${dirname}" "${filename}-active_ACS_Demo_verify" "Invalid SCmachineMode"
        exit 1
    fi
}

# Print out log file location
# Can be run multiple times
print_log_path () {
    echo_notice "${dirname}" "${filename}-print_log_path" "acs_demo_server_log_dir: ${acs_demo_server_log_dir}"
    echo_notice "${dirname}" "${filename}-print_log_path" "acs_demo_client_log_dir: ${acs_demo_client_log_dir}"
    echo_notice "${dirname}" "${filename}-print_log_path" "swtpm_bios_log_dir: ${swtpm_bios_log_dir}"
    echo_notice "${dirname}" "${filename}-print_log_path" "acs_demo_verify_tpm2bios_log_dir: ${acs_demo_verify_tpm2bios_log_dir}"
    echo_notice "${dirname}" "${filename}-print_log_path" "ima_sig_log_dir: ${ima_sig_log_dir}"
    echo_notice "${dirname}" "${filename}-print_log_path" "acs_demo_verify_imasig_log_dir: ${acs_demo_verify_imasig_log_dir}"
    echo_notice "${dirname}" "${filename}-print_log_path" "acs_demo_verify_client_log_dir: ${acs_demo_verify_client_log_dir}"
}

# Open all log files in new terminal tabs
# Can be run multiple times
# Usage: open_all_logs <mod_num>
# Input $1: 1 - open in terminal, 2 - open in tmux
open_all_logs () {
    if [ $1 == 1 ]; then
        newt () {
            lc1="source ${current_dir}/../common/functions.sh"
            lc2="echo_notice \"setup_ibmtpm\" \"setup-open_all_logs\" \"tailling log file: $1\""
            lc3="tail -f $1 -n $2"
            if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ] || [ $install_platform -eq 5 ]; then
                newGterm "$(basename -- $1)" "$bash_gflag" "$lc1; $lc2; $lc3" 1
            else
                newLXterm "$(basename -- $1)" "$lc1; $lc2; $lc3" 1
            fi
        }
        newt "${acs_demo_server_log_dir}" ${log4j_line_number}
        newt "${acs_demo_client_log_dir}" ${log4j_line_number}
        newt "${swtpm_bios_log_dir}" ${log4j_line_number}
        newt "${acs_demo_verify_tpm2bios_log_dir}" ${log4j_line_number}
        newt "${ima_sig_log_dir}" ${log4j_line_number}
        newt "${acs_demo_verify_imasig_log_dir}" ${log4j_line_number} # Gibberish
        newt "${acs_demo_verify_client_log_dir}" ${log4j_line_number} # Gibberish
    elif [ $1 == 2 ]; then
        tmux kill-server
        lc1="source ${current_dir}/../common/functions.sh"
        lc2="sudo -u $user tmux"
        if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ] || [ $install_platform -eq 5 ]; then
            newGterm "ACS LOGS" "$bash_gflag" "$lc1; $lc2" 1
        else
            newLXterm "ACS LOGS" "$lc1; $lc2" 1
        fi
        settitle () {
            printf '\033]2;%s\033\\' "$1"
        }
        tssession=$user
        err_conti_exec "tmux -2 new-session -d -s $tssession" "${dirname}" "setup-open_all_logs"
        tmux new-window -t $tssession:1 -n 'log4j'
        tmux split-window -h
        tmux split-window -h
        tmux select-layout even-horizontal
        tmux select-pane -t 2
        tmux split-window -v
        tmux select-pane -t 1
        tmux split-window -v
        tmux select-pane -t 0
        tmux split-window -v
        tmux select-pane -t 0
        settitle "ACS Demo Server"
        tmux send-keys 'htop' Enter
        tmux select-pane -t 1
        settitle "ACS Demo Client"
        tmux send-keys 'htop' Enter
        tmux select-pane -t 2
        settitle "SWTPM BIOS"
        tmux send-keys 'htop' Enter
        tmux select-pane -t 3
        settitle "ACS Demo verify TPM2BIOS"
        tmux send-keys 'htop' Enter
        tmux select-pane -t 4
        settitle "IMASIG"
        tmux send-keys 'htop' Enter
        tmux select-pane -t 5
        settitle "ACS Demo verify IMASIG"
        tmux send-keys 'htop' Enter
        tmux select-window -t $tssession:1
        tmux -2 attach-session -t $tssession
    else
        echo_warn "${dirname}" "${filename}-open_all_logs" "Invalid input"
        exit 1
    fi
}

echo_notice "${dirname}" "${filename}" "Running setup script ..."

# look for option preset (install_platform & config_file)
if [ -z ${install_platform+x} ]; then
    found_install_platform=false
    preset_list=($(ls $current_dir | grep -E 'ip[0-9]+')) || { echo -e "No preset option found for install_platform"; }
    preset_file=${preset_list[0]}
    install_platform=${preset_file:2}
    if ! [ -z $install_platform ]; then
        found_install_platform=true
    fi
else
    found_install_platform=true
fi
found_config_file=false
preset_list=($(ls $current_dir | grep -E 'cf[0-9]+')) || { echo -e "No preset option found for config_file"; }
preset_file=${preset_list[0]}
config_file=${preset_file:2}
if ! [ -z $config_file ]; then
    found_config_file=true
fi

# no preset option found, take user input
if ! [ $found_install_platform == true ]; then
    echo -e "Please select the platform to install:"
    echo -e "1. Ubuntu 18.04 VM"
    echo -e "2. Raspbian Bullseye 2022-07-01 5.1 Kernel Debian i386 VM"
    echo -e "3. Raspbian Bullseye 2023-05-03 6.x Kernel Debian arm64 Raspberry Pi 4"
    echo -e "4. Ubuntu 22.04.3 on Raspberry Pi 4 B"
    echo -e "5. Jetson Nano"
    echo -e "Else. Exit"
    read -rsn1 -p "Select: " install_platform
    echo -e "$install_platform\n"
    if [[ $install_platform -lt 1 ]] || [[ $install_platform -gt 5 ]]; then
        echo -e "\nExiting ..."
        exit 1
    fi
fi
config_file_list=($(ls $current_dir/config_files))
config_file_num=${#config_file_list[@]}
if ! [ $found_config_file == true ]; then
    echo -e "Please select the config file to use:"
    for index in "${!config_file_list[@]}"; do
        echo -e "$((index+1)). ${config_file_list[index]}"
    done
    echo -e "Else. Exit"
    read -rsn1 -p "Select: " config_file
    echo -e "$config_file\n"
    if [[ $config_file -lt 1 ]] || [[ $config_file -gt $config_file_num ]]; then
        echo -e "\nExiting ..."
        exit 1
    fi
fi
echo -e "install_platform: $install_platform\n"
echo -e "config_file: ${config_file_list[$((config_file-1))]}\n"
load_preset "$current_dir/config_files/${config_file_list[$((config_file-1))]}"
cp $current_dir/config_files/${config_file_list[$((config_file-1))]} $current_dir/config.ini

dn_ibmtss="ibmtss"
dn_ibmtpm="ibmtpm"
dn_ibmacs="ibmacs"
fn_ibmtss="${dn_ibmtss}${ibmtss_ver}.tar.gz"
fn_ibmtpm="${dn_ibmtpm}${ibmtpm_ver}.tar.gz"
fn_ibmacs="${dn_ibmacs}${ibmacs_ver}.tar.gz"
sym_link_ibmtss="${base_dir}/${dn_ibmtss}"
sym_link_ibmtpm="${base_dir}/${dn_ibmtpm}"
sym_link_ibmacs="${base_dir}/${dn_ibmacs}"
path_ibmtss="${sym_link_ibmtss}${ibmtss_ver}"
path_ibmtpm="${sym_link_ibmtpm}${ibmtpm_ver}"
path_ibmacs="${sym_link_ibmacs}${ibmacs_ver}"
path_NV="${sym_link_ibmtpm}/src/NVChip"
tss_cert_rootcert_dir="${sym_link_ibmtss}/utils/certificates"
acs_demo_url_A="${acs_demo_server_ip}:${acs_demo_server_port}/acs"
acs_demo_url_B="${acs_demo_server_ip}/acs"
tpm2_nvread="/usr/local/bin/tpm2_nvread" # Required setup_optiga installed
acs_demo_server_log_dir="${sym_link_ibmacs}/serverenroll.log4j"
acs_demo_client_log_dir="${sym_link_ibmacs}/clientenroll.log4j"
swtpm_bios_log_dir="${sym_link_ibmacs}/tpm2bios.log"
acs_demo_verify_tpm2bios_log_dir="${sym_link_ibmacs}/b.log4j"
ima_sig_log_dir="${sym_link_ibmacs}/imasig.log"
acs_demo_verify_imasig_log_dir="${sym_link_ibmacs}/i.log4j"
acs_demo_verify_client_log_dir="${sym_link_ibmacs}/client.log4j"

if [[  $install_req            == 1 ]]; then install_req;                 fi
if [[  $setup_ibmtpmtss_env    == 1 ]]; then setup_ibmtpmtss_env;         fi
if [[  $compile_ibmtpmtss      == 1 ]]; then compile_ibmtpmtss;           fi
if [[  $setup_ibmswtpm_env     == 1 ]]; then setup_ibmswtpm_env;          fi
if [[  $compile_ibmswtpm       == 1 ]]; then compile_ibmswtpm;            fi
if [[  $setup_ibmacs_env       == 1 ]]; then setup_ibmacs_env;            fi
if [[  $compile_ibmacs         == 1 ]]; then compile_ibmacs;              fi
if [[  $open_demo_webpage      == 1 ]]; then open_demo_webpage;           fi
if [[  $generate_CA            == 1 ]]; then generate_CA;                 fi
if [[  $activate_TPM_server    == 1 ]]; then activate_TPM_server;         fi
if [[  $activate_TPM_client    == 1 ]]; then activate_TPM_client;         fi
if [[  $generate_EK            == 1 ]]; then generate_EK;                 fi
if [[  $retrieve_EK            == 1 ]]; then retrieve_EK;                 fi
if [[  $set_acs_sql_setting    == 1 ]]; then set_acs_sql_setting;         fi
if [[  $active_ACS_Demo_Server == 1 ]]; then active_ACS_Demo_Server;      fi
if [[  $active_ACS_Demo_Client == 1 ]]; then active_ACS_Demo_Client;      fi
if [[  $active_ACS_Demo_verify == 1 ]]; then active_ACS_Demo_verify;      fi
if [[  $print_log_path         == 1 ]]; then print_log_path;              fi
if [[  $open_all_logs          == 1 ]]; then open_all_logs 1;             fi

clear_preset

echo_notice "${dirname}" "${filename}" "Setup complete"
