#!/bin/bash

source "../common/function.sh"
source "./function_ibmtpm.sh"
load_preset "./config.ini"

current_dir=$(pwd)
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
acs_demo_url="${acs_demo_server_ip}:${acs_demo_server_port}/acs"
acs_demo_server_log_dir="${sym_link_ibmacs}/serverenroll.log4j"
acs_demo_client_log_dir="${sym_link_ibmacs}/clientenroll.log4j"
swtpm_bios_log_dir="${sym_link_ibmacs}/tpm2bios.log"
acs_demo_verify_tpm2bios_log_dir="${sym_link_ibmacs}/b.log4j"
ima_sig_log_dir="${sym_link_ibmacs}/imasig.log"
acs_demo_verify_imasig_log_dir="${sym_link_ibmacs}/i.log4j"
acs_demo_verify_client_log_dir="${sym_link_ibmacs}/client.log4j"
MariaDB_dir="${HOME}/MariaDB"

# Install requirements for development, building, and testing
# Download ibmtss, ibmtpm, and ibmacs from sourceforge
# Extract ibmtss, ibmtpm, and ibmacs
# Only need to setup once (can re-run)
install_req () {
    echo_notice "setup_ibmtpm" "setup-install_req" "Creating directories ..."
    mkdir "${path_ibmtss}"
    mkdir "${path_ibmtpm}"
    mkdir "${path_ibmacs}"

    echo_notice "setup_ibmtpm" "setup-install_req" "Downloading IBMTPMTSS ..."
    wget $wget_gflag "https://sourceforge.net/projects/ibmtpm20tss/files/${fn_ibmtss}/download" -O "${path_ibmtss}/${fn_ibmtss}"

    echo_notice "setup_ibmtpm" "setup-install_req" "Downloading IBMSWTPM ..."
    wget $wget_gflag "https://sourceforge.net/projects/ibmswtpm2/files/${fn_ibmtpm}/download" -O "${path_ibmtpm}/${fn_ibmtpm}"

    echo_notice "setup_ibmtpm" "setup-install_req" "Downloading IBMACS ..."
    wget $wget_gflag "https://sourceforge.net/projects/ibmtpm20acs/files/${fn_ibmacs}/download" -O "${path_ibmacs}/${fn_ibmacs}"

    echo_notice "setup_ibmtpm" "setup-install_req" "Extracting IBMTPMTSS ..."
    tar $tar_gflag "${path_ibmtss}/${fn_ibmtss}" -C ${path_ibmtss}

    echo_notice "setup_ibmtpm" "setup-install_req" "Extracting IBMSWTPM ..."
    tar $tar_gflag "${path_ibmtpm}/${fn_ibmtpm}" -C ${path_ibmtpm}

    echo_notice "setup_ibmtpm" "setup-install_req" "Extracting IBMACS ..."
    tar $tar_gflag "${path_ibmacs}/${fn_ibmacs}" -C ${path_ibmacs}
}

# Set environment variables for ibmtss, and create symbolic link to ibmtss
# Can re-run to update environment variables to switch between physical TPM and software TPM
# Only need to setup once (can re-run)
setup_ibmtpmtss_env () {
    echo_notice "setup_ibmtpm" "setup-setup_ibmtpmtss_env" "Setting path env variable ..."
    if [ $TPMMode == 1 ]; then
        # for Pysical TPM
        export TPM_INTERFACE_TYPE=dev
    elif [ $TPMMode == 2 ]; then
        # for Software TPM
        export TPM_INTERFACE_TYPE=socsim
    else 
        echo -e "${BOLD}${RED}Invalid TPMMode${NC}"
        echo_warn "setup_ibmtpm" "setup-setup_ibmtpmtss_env" "Invalid TPMMode"
        exit 1
    fi
    export TPM_COMMAND_PORT="${tpm_command_port}"

    echo_notice "setup_ibmtpm" "setup-setup_ibmtpmtss_env" "Creating symbolic link to ${path_ibmtss} ..."
    ln -s "${path_ibmtss}" "${base_dir}/ibmtss"
}

# Compile ibmtss
# Can be run multiple times for code adjustment
compile_ibmtpmtss () {
    echo_notice "setup_ibmtpm" "setup-compile_ibmtpmtss" "Cleaning up with make ..."
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
        echo_warn "setup_ibmtpm" "setup-compile_ibmtpmtss" "Invalid verMode"
        exit 1
    fi

    echo_notice "setup_ibmtpm" "setup-compile_ibmtpmtss" "Compiling IBMTSS ..."
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmtss}/utils/"
        make $make_gflag -f makefiletpm20
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmtss}/utils/"
        make $make_gflag -f makefiletpmc
        cd "${path_ibmtss}/utils12/"
        make $make_gflag -f makefiletpmc
    else 
        echo_warn "setup_ibmtpm" "setup-compile_ibmtpmtss" "Invalid verMode"
        exit 1
    fi
}

# Create symbolic link to ibmswtpm
# Only need to setup once (can re-run)
setup_ibmswtpm_env () {
    echo_notice "setup_ibmtpm" "setup-setup_ibmswtpm_env" "Creating symbolic link to ${path_ibmtpm} ..."
    ln -s "${path_ibmtpm}" "${base_dir}/ibmtpm"
}

# Compile ibmswtpm
# Can be run multiple times for code adjustment
compile_ibmswtpm () {
    echo_notice "setup_ibmtpm" "setup-compile_ibmswtpm" "Cleaning up with make ..."
    cd "${path_ibmtpm}/src/"
    make $make_gflag clean

    echo_notice "setup_ibmtpm" "setup-compile_ibmswtpm" "Compiling IBMTPM ..."
    cd "${path_ibmtpm}/src/"
    make $make_gflag
}

# Install requirements for ibmacs, create mysql database, set environment variables, link directories, and generate directory for webpage
# Only need to setup once (can re-run)
setup_ibmacs_env () {
    echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Installing IBMACS dependencies ..."
    if [ $acsMode == 1 ]; then
        # for Server
        if [ $install_platform -eq 1 ]; then
            aptins "libjson-c-dev apache2 php php-dev php-mysql mysql-server libmysqlclient-dev libssl-dev"
        elif [ $install_platform -eq 2 ]; then
            aptins "libjson-c-dev apache2 php php-dev php-mysql mariadb-server libmariadb-dev-compat libssl-dev"
            # aptins "libmariadb-dev"
            #aptins "libjson-c-dev apache2 php php-dev php-mysql libssl-dev"
        elif [ $install_platform -eq 3 ]; then
            aptins "libjson-c-dev"
            aptins "apache2"
            aptins "php"
            aptins "php-dev"
            aptins "php-mysql"
            aptins "mariadb-server"
            aptins "libmariadb-dev"
            aptins "libmariadb-dev-compat"
        elif [ $install_platform -eq 4 ]; then
            aptins "libjson-c-dev apache2 php php-dev php-mysql mysql-server libmysqlclient-dev libssl-dev"
        else
            echo_error "setup_ibmtpm" "setup-setup_ibmacs_env" "Invalid install_platform" 1
        fi
    elif [ $acsMode == 2 ]; then
        # for Client
        aptins "libjson-c-dev libssl-dev"
    else 
        echo_warn "setup_ibmtpm" "setup-setup_ibmacs_env" "Invalid acsMode"
        exit 1
    fi

    # ACS source platform adaption
    if [ $install_platform -eq 1 ]; then
        :
    elif [ $install_platform -eq 2 ]; then
        echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Adding stdbool.h to IBMACS/acs/commonjson.c"
        sed -i '39 i #include <stdbool.h>' "${path_ibmacs}/acs/commonjson.c"

        echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Replacing \"FALSE\" with \"false\" in IBMACS/acs/commonjson.c"
        sed -i 's/FALSE/false/g' "${path_ibmacs}/acs/commonjson.c"

        echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Replacing all mysql/mysql.h with mariadb/mysql.h in all files"
        for file in $(grep -rl "mysql/mysql.h" "${path_ibmacs}/acs/"); do
            sed -i 's/mysql\/mysql.h/mariadb\/mysql.h/g' $file
        done

        #echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Compiling MariaDB from source ..."
        #cd "$MariaDB_dir"
        #git clone https://github.com/MariaDB/mariadb-connector-c.git
        #mkdir build && cd build
        #cmake ../mariadb-connector-c/ -DCMAKE_INSTALL_PREFIX=/usr
        #make -j$(nproc) $make_gflag
        #sudo make install $make_gflag
    elif [ $install_platform -eq 3 ]; then
        :
    else
        echo_error "setup_ibmtpm" "setup-setup_ibmacs_env" "Invalid install_platform" 1
    fi

    if [ $acsMode == 1 ]; then
        # https://stackoverflow.com/questions/24270733/automate-mysql-secure-installation-with-echo-command-via-a-shell-script
        echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Setting database ..."
        cd "${path_ibmacs}/acs/"
        mysql -Bse "CREATE DATABASE IF NOT EXISTS ${mysql_database};"
        mysql -Bse "CREATE USER IF NOT EXISTS '${mysql_user}'@'${acs_demo_server_ip}' IDENTIFIED BY '${mysql_password}';"
        mysql -Bse "GRANT ALL PRIVILEGES ON ${mysql_database}.* TO '${mysql_user}'@'${acs_demo_server_ip}';"
        mysql -D ${mysql_database} < "${path_ibmacs}/acs/dbinit.sql"
    fi

    echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Setting include path ..."
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"
    export PATH="${PATH}:${path_ibmtss}/utils:${path_ibmtss}/utils12"

    echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Creating symbolic link to ${path_ibmacs} ..."
    ln -s "${path_ibmacs}/acs" "${base_dir}/ibmacs"

    echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Setting html directory ..."
    mkdir -p ${html_dir}
    chown root ${html_dir}
    chgrp root ${html_dir}
    chmod 777 ${html_dir}

    echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Creating symbolic link to ${c_json_lib_dir} ..."
    ln -s "${c_json_lib_dir}" "${c_json_lib_link_dir}"

    echo_notice "setup_ibmtpm" "setup-setup_ibmacs_env" "Setting include path ..."
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
        echo_warn "setup_ibmtpm" "setup-setup_ibmacs_env" "Invalid verMode"
        exit 1
    fi
}

# Compile ibmacs
# Can be run multiple times for code adjustment
compile_ibmacs () {
    echo_notice "setup_ibmtpm" "setup-compile_ibmacs" "Compiling IBMACS and setting include path ..."
    if [ $verMode == 1 ]; then
        # for TPM 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils"
        export LIBRARY_PATH="${path_ibmtss}/utils"
        make $make_gflag clean
        make $make_gflag
    elif [ $verMode == 2 ]; then
        # for TPM 1.2 & 2.0
        cd "${path_ibmacs}/acs/"
        export CPATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        export LIBRARY_PATH="${path_ibmtss}/utils:${path_ibmtss}/utils12"
        if [ $acsMode == 1 ]; then
            # for Server
            make $make_gflag -f makefiletpmc clean
            make $make_gflag -f makefiletpmc
        elif [ $acsMode == 2 ]; then
            # for Client
            make $make_gflag -f makefiletpm12 clean
            make $make_gflag -f makefiletpm12
            make $make_gflag -f makefiletpmc clean
            make $make_gflag -f makefiletpmc
        else 
            echo_warn "setup_ibmtpm" "setup-compile_ibmacs" "Invalid acsMode"
            exit 1
        fi
    else 
        echo_warn "setup_ibmtpm" "setup-compile_ibmacs" "Invalid verMode"
        exit 1
    fi
}

# Open demo webpage with firefox
# Can be run multiple times
open_demo_webpage () {
    echo_notice "setup_ibmtpm" "setup-open_demo_webpage" "Opening demo webpage with new terminal ..."
    launch_cmd1="echo -e \"setup_ibmtpm\" \"setup-open_demo_webpage\" \"Opening demo webpage with new terminal ...\n\""
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ]; then
        launch_cmd2="sudo -u $user bash -c \"firefox --new-tab -url ${acs_demo_url} --new-tab -url ${repo_url} &\""
    else
        launch_cmd2="sudo -u $user bash -c \"chromium-browser --new-window -url ${acs_demo_url} --new-window -url ${repo_url} &\""
    fi
    launch_cmd3="echo -e \"\nctrl+c to exit\n\"; sleep infinity"
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ]; then
        gnome-terminal -t "FireFox Browser" --active -- bash $bash_gflag -c "${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}"
    else
        lxterminal -t "Chromium Browser" -e "${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}" &
    fi

    echo_notice "setup_ibmtpm" "setup-open_demo_webpage" "Opening demo webpage with new terminal ..."
    lc1="source ${current_dir}/../common/function.sh"
    lc2="echo_notice \"setup_ibmtpm\" \"setup-open_demo_webpage\" \"Opening demo webpage with new terminal ...\""
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ]; then
        lc3="sudo -u $user bash -c \"firefox --new-tab -url ${acs_demo_url} --new-tab -url ${repo_url} &\""
        newGterm "FireFox Browser" "$bash_gflag" "$lc1; $lc2; $lc3" 1
    else
        lc3="sudo -u $user bash -c \"chromium-browser --new-window -url ${acs_demo_url} --new-window -url ${repo_url} &\""
        newLxterm "Chromium Browser" "$lc1; $lc2; $lc3" 1
    fi
}

# Generate CA certificate and key
# Only need to setup once (can re-run)
generate_CA () {
    echo_warn "setup_ibmtpm" "setup-generate_CA" "Function not implemented"
    echo_warn "setup_ibmtpm" "setup-generate_CA" "Refer to ${sym_link_ibmacs}/README.txt line 171 for steps."
    echo_notice "setup_ibmtpm" "setup-generate_CA" "Generated CAs in ${sym_link_ibmtss}/utils ......"
    ls "${sym_link_ibmtss}/utils/"*.pem
    echo_notice "setup_ibmtpm" "setup-generate_CA" "Generated CAs in ${sym_link_ibmacs} ......"
    ls "${sym_link_ibmacs}/"*.pem
}

# Activate TPM Server in new terminal
# Only need to setup once (can re-run)
activate_TPM_server () {
    TPM_server_executed=1
    # apply TPMMode for ibmtss
    setup_ibmtpmtss_env

    echo_notice "setup_ibmtpm" "setup-activate_TPM_server" "Starting TPM simulator (SWTPM/vTPM/server) on new temrinal ..."
    cd "${sym_link_ibmtpm}/src/"
    launch_cmd1="echo -e \"setup_ibmtpm\" \"setup-activate_TPM_server\" \"Starting TPM simulator (server) on new temrinal ...\n\""
    launch_cmd2="./tpm_server"
    launch_cmd3="echo -e \"\nctrl+c to exit\n\"; sleep infinity"
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ]; then
        gnome-terminal -t "TPM SERVER" --active -- bash $bash_gflag -c "${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}"
    else
        lxterminal -t "TPM SERVER" -e "${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}" &
    fi
}

# Activate TPM Client in current terminal
# Only need to setup once (can re-run)
activate_TPM_client () {
    TPM_client_executed=1
    echo_notice "setup_ibmtpm" "setup-activate_TPM_client" "Starting TPM simulator (SWTPM/vTPM/client) on new temrinal ..."
    cd "${sym_link_ibmtss}/utils/"
    launch_cmd1="echo -e \"setup_ibmtpm\" \"setup-activate_TPM_client\" \"Starting TPM simulator (client) on new temrinal ...\n\""
    launch_cmd2="./powerup"
    launch_cmd3="./startup"
    launch_cmd4="echo -e \"\nctrl+c to exit\n\"; sleep infinity"
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ]; then
        gnome-terminal -t "TPM CLIENT" --active -- bash $bash_gflag -c "${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}; ${launch_cmd4}"
    else
        lxterminal -t "TPM CLIENT" -e "${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}; ${launch_cmd4}" &
    fi
}

# Create EK certificate and key, activated TPM on new terminal
# Only need to setup once (can re-run)
generate_EK () {
    activate_TPM_server

    activate_TPM_client

    echo_notice "setup_ibmtpm" "setup-generate_EK" "Backing up NVChip ......"
    cp "${path_NV}" "${path_NV}.bak"

    echo_notice "setup_ibmtpm" "setup-generate_EK" "Generating RSAEK and load into NV ......"
    ./createekcert -rsa 2048 -cakey $RSAEK_cert -capwd rrrr -v

    echo_notice "setup_ibmtpm" "setup-generate_EK" "Generating ECCEK and load into NV ......"
    ./createekcert -ecc nistp256 -cakey $ECCEK_cert -capwd rrrr -caalg ec -v
}

# Retrieve hardware NVChip
# Only need to setup once (can re-run)
retrieve_hardware_NV () {
    echo_warn "setup_ibmtpm" "setup-retrieve_hardware_NV" "Function not implemented"
}

# Set ACS MYSQL setting
# Only need to setup once (can re-run)
set_acs_sql_setting () {
    echo_notice "setup_ibmtpm" "setup-set_acs_sql_setting" "Setting ACS MYSQL Setting ..."
    export ACS_SQL_USERID="${mysql_user}"
    export ACS_SQL_PASSWORD="${mysql_password}"

    if [ $force_acs_sql_setting == 1 ]; then
        echo_warn "setup_ibmtpm" "setup-set_acs_sql_setting" "Forcing ACS MySQL Setting ..."
        cp "${html_dir}/dbconnect.php" "${html_dir}/dbconnect.php.bak"
        sed -i 's@($acs_sql_host, $acs_sql_userid, $acs_sql_password, $acs_sql_database)@'"(\"${acs_demo_server_ip}\", \"${mysql_user}\", \"${mysql_password}\", \"${mysql_database}\")"'@' "${html_dir}/dbconnect.php"
    else
        echo_warn "setup_ibmtpm" "setup-set_acs_sql_setting" "Not Forcing ACS MySQL Setting ..."
    fi
}

# Active ACS Demo Server
# Can be run multiple times
active_ACS_Demo_Server () {
    if [ $SCmachineMode == 1 ]; then
        echo_notice "setup_ibmtpm" "setup-active_ACS_Demo_Server" "Activating ACS Demo on same machine ..."
        mkdir "${tpm_data_dir}"
        export TPM_DATA_DIR="${tpm_data_dir}"
    elif [ $SCmachineMode == 2 ]; then
        echo_notice "setup_ibmtpm" "setup-active_ACS_Demo_Server" "Activating ACS Demo on different machine ..."
    else 
        echo_warn "setup_ibmtpm" "setup-active_ACS_Demo_Server" "Invalid SCmachineMode"
        exit 1
    fi

    echo_notice "setup_ibmtpm" "setup-active_ACS_Demo_Server" "Replacing path in ${tss_cert_rootcert_dir}/rootcerts.txt ..."
    cp "${tss_cert_rootcert_dir}/rootcerts.txt" "${tss_cert_rootcert_dir}/rootcerts.txt.bak"
    sed -i "s/\/home\/kgold\/tss2/\\${base_dir}\/${dn_ibmtss}/g" "${tss_cert_rootcert_dir}/rootcerts.txt"
    export ACS_PORT="${acs_port}"

    set_acs_sql_setting

    echo_notice "setup_ibmtpm" "setup-active_ACS_Demo_Server" "Activating ACS Demo on new terminal ..."
    cd ${path_ibmacs}/acs
    launch_cmd1="source ${current_dir}/../common/function.sh"
    launch_cmd2="log_date_time \"./server -v -root ${tss_cert_rootcert_dir}/rootcerts.txt -imacert imakey.der\" \"$log4j_time_format\" \"${acs_demo_server_log_dir}\" \"default\""
    launch_cmd3="echo -e \"\nctrl+c to exit\n\"; sleep infinity"
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ]; then
        gnome-terminal -t "ACS SERVER" --active -- bash $bash_gflag -c "${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}"
    else
        lxterminal -t "ACS SERVER" -e "${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}" &
    fi
}

# Active ACS Demo Client
# Can be run multiple times
active_ACS_Demo_Client () {
    cd "${path_ibmacs}/acs"
    if [ $acsClientMode == 1 ]; then
        echo_notice "setup_ibmtpm" "setup-active_ACS_Demo_Client" "Activating ACS Demo on local machine ..."
        log_date_time "./clientenroll -alg rsa -v -ho ${acs_demo_server_ip} -co akcert.pem" "$log4j_time_format" "${acs_demo_client_log_dir}" "default"
    elif [ $acsClientMode == 2 ]; then
        echo_notice "setup_ibmtpm" "setup-active_ACS_Demo_Client" "Activating ACS Demo on remote machine ..."
        log_date_time "./clientenroll -alg ec -v -ho ${acs_demo_server_ip} -ma ${acs_demo_client_ip} -co akeccert.pem" "$log4j_time_format" "${acs_demo_client_log_dir}" "default"
    else 
        echo_warn "setup_ibmtpm" "setup-active_ACS_Demo_Client" "Invalid acsClientMode"
        exit 1
    fi
}

# Active ACS Demo verify
# Can be run multiple times
active_ACS_Demo_verify () {
    cd "${path_ibmacs}/acs"
    if [ $TPMMode == 1 ]; then
        # for Pysical TPM
        echo_notice "setup_ibmtpm" "setup-active_ACS_Demo_verify" "Ignore when working with Physical TPM"
    elif [ $TPMMode == 2 ]; then
        # for Software TPM
        cd "${sym_link_ibmacs}"
        echo_notice "setup_ibmtpm" "setup-active_ACS_Demo_verify" "Checking TPM2BIOS.LOG ..."
        log_date_time "${sym_link_ibmtss}/utils/eventextend -if ${swtpm_bios_log_dir} -tpm -v" "$log4j_time_format" "${acs_demo_verify_tpm2bios_log_dir}" "default"

        echo_notice "setup_ibmtpm" "setup-active_ACS_Demo_verify" "Checking IMASIG.LOG ..."
        log_date_time "${sym_link_ibmtss}/utils/imaextend -if ${ima_sig_log_dir} -le -v" "$log4j_time_format" "${acs_demo_verify_imasig_log_dir}" "default"

        if [ $acsClientMode == 1 ]; then
            # for Local
            log_date_time "${sym_link_ibmacs}/client -alg rsa -ifb ${swtpm_bios_log_dir} -ifi ${ima_sig_log_dir} -ho ${acs_demo_server_ip} -v" "$log4j_time_format" "${acs_demo_verify_client_log_dir}" "default"
        elif [ $acsClientMode == 2 ]; then
            # for Remote
            log_date_time "${sym_link_ibmacs}/client -alg ec -ifb ${swtpm_bios_log_dir} -ifi ${ima_sig_log_dir} -ho ${acs_demo_server_ip} -v -ma ${acs_demo_client_ip}" "$log4j_time_format" "${acs_demo_verify_client_log_dir}" "default"
        else 
            echo_warn "setup_ibmtpm" "setup-active_ACS_Demo_verify" "Invalid acsClientMode"
            exit 1
        fi
    else 
        echo_warn "setup_ibmtpm" "setup-active_ACS_Demo_verify" "Invalid TPMMode"
        exit 1
    fi
}

# Print out log file location
# Can be run multiple times
print_log_path () {
    echo_notice "setup_ibmtpm" "setup-print_log_path" "acs_demo_server_log_dir: ${acs_demo_server_log_dir}"
    echo_notice "setup_ibmtpm" "setup-print_log_path" "acs_demo_client_log_dir: ${acs_demo_client_log_dir}"
    echo_notice "setup_ibmtpm" "setup-print_log_path" "swtpm_bios_log_dir: ${swtpm_bios_log_dir}"
    echo_notice "setup_ibmtpm" "setup-print_log_path" "acs_demo_verify_tpm2bios_log_dir: ${acs_demo_verify_tpm2bios_log_dir}"
    echo_notice "setup_ibmtpm" "setup-print_log_path" "ima_sig_log_dir: ${ima_sig_log_dir}"
    echo_notice "setup_ibmtpm" "setup-print_log_path" "acs_demo_verify_imasig_log_dir: ${acs_demo_verify_imasig_log_dir}"
    echo_notice "setup_ibmtpm" "setup-print_log_path" "acs_demo_verify_client_log_dir: ${acs_demo_verify_client_log_dir}"
}

# Open all log files in new terminal tabs
# Can be run multiple times
open_all_logs () {
    newt () {
        lcmd0="echo -e \"\nctrl+c to exit\n\"; sleep infinity"
        lcmd1="echo \"tailling log file: $1\""
        lcmd2="tail -f $1 -n $2"
        if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ]; then
            gnome-terminal -t "$(basename -- $1)" --active -- bash $bash_gflag -c "${lcmd1}; ${lcmd2}; ${lcmd0}"
        else
            lxterminal -t "$(basename -- $1)" -e "${lcmd1}; ${lcmd2}; ${lcmd0}" &
        fi
    }
    newt "${acs_demo_server_log_dir}" ${log4j_line_number}
    newt "${acs_demo_client_log_dir}" ${log4j_line_number}
    newt "${swtpm_bios_log_dir}" ${log4j_line_number}
    newt "${acs_demo_verify_tpm2bios_log_dir}" ${log4j_line_number}
    newt "${ima_sig_log_dir}" ${log4j_line_number}
    newt "${acs_demo_verify_imasig_log_dir}" ${log4j_line_number}
    newt "${acs_demo_verify_client_log_dir}" ${log4j_line_number}
}

echo_notice "setup_ibmtpm" "setup" "Running setup script ..."

if [ $install_req            == 1 ]; then install_req;                 fi
if [ $setup_ibmtpmtss_env    == 1 ]; then setup_ibmtpmtss_env;         fi
if [ $compile_ibmtpmtss      == 1 ]; then compile_ibmtpmtss;           fi
if [ $setup_ibmswtpm_env     == 1 ]; then setup_ibmswtpm_env;          fi
if [ $compile_ibmswtpm       == 1 ]; then compile_ibmswtpm;            fi
if [ $setup_ibmacs_env       == 1 ]; then setup_ibmacs_env;            fi
if [ $compile_ibmacs         == 1 ]; then compile_ibmacs;              fi
if [ $open_demo_webpage      == 1 ]; then open_demo_webpage;           fi
if [ $generate_CA            == 1 ]; then generate_CA;                 fi
if [ $activate_TPM_server    == 1 ]; then activate_TPM_server;         fi
if [ $activate_TPM_client    == 1 ]; then activate_TPM_client;         fi
if [ $generate_EK            == 1 ]; then generate_EK;                 fi
if [ $retrieve_hardware_NV   == 1 ]; then retrieve_hardware_NV;        fi
if [ $set_acs_sql_setting    == 1 ]; then set_acs_sql_setting;         fi
if [ $active_ACS_Demo_Server == 1 ]; then active_ACS_Demo_Server;      fi
if [ $active_ACS_Demo_Client == 1 ]; then active_ACS_Demo_Client;      fi
if [ $active_ACS_Demo_verify == 1 ]; then active_ACS_Demo_verify;      fi
if [ $print_log_path         == 1 ]; then print_log_path;              fi
if [ $open_all_logs          == 1 ]; then open_all_logs;               fi

clear_preset

echo_notice "setup_ibmtpm" "setup" "Setup complete"
