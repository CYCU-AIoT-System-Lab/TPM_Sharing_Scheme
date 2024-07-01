#/bin/bash

source "./functions.sh"
source "./function_common.sh"
load_preset "./config.ini"

dirname="common"
filename="setup"

# Functions

build_cmake () {
    echo_notice "${dirname}" "${filename}" "Building ${BOLD}${GREEN}cmake${END}..."
    err_conti_exec "sudo mkdir -p $cmake_dir" "common" "setup_build_cmake"
    cd $cmake_dir
    sudo wget $wget_gflag "https://cmake.org/files/v${cmake_ver}/cmake-${cmake_ver}.${cmake_build}.tar.gz"
    sudo tar -zxf "cmake-${cmake_ver}.${cmake_build}.tar.gz"
    cd "cmake-${cmake_ver}.${cmake_build}"
    sudo ./bootstrap --parallel=$(nproc)
    sudo make $make_gflag -j$(nproc)
    sudo $sudo_gflag make $make_gflag install -j$(nproc)
    cmake --version
}

build_valgrind () {
    echo_notice "${dirname}" "${filename}" "Building ${BOLD}${GREEN}valgrind${END}..."
    err_conti_exec "sudo mkdir -p $valgrind_dir" "common" "setup_build_valgrind"
    cd $valgrind_dir
    sudo wget $wget_gflag "https://sourceware.org/pub/valgrind/valgrind-${valgrind_ver}.tar.bz2"
    sudo tar xfj "valgrind-${valgrind_ver}.tar.bz2"
    cd "valgrind-${valgrind_ver}"
    sudo ./configure
    sudo make $make_gflag -j$(nproc)
    sudo $sudo_gflag make $make_gflag install -j$(nproc)
}

build_libssl () {
    libssl_build="openssl-${libssl_ver}.tar.gz"
    libssl_build_name="$(basename $libssl_build .tar.gz)"
    libssl_path_load_sh="/etc/profile.d/openssl.sh"
    err_conti_exec "sudo mkdir $libssl_dir" "common" "setup_build_libssl"
    cd $libssl_dir
    sudo wget "https://www.openssl.org/source/${libssl_build}"
    sudo tar xfz $libssl_build --directory $libssl_dir
    export LD_LIBRARY_PATH="/opt/openssl/lib"
    cd "${libssl_dir}/${libssl_build_name}"
    sudo ./config --prefix=$libssl_dir --openssldir="${libssl_dir}/ssl"
    sudo make -j$(nproc)
    sudo make test -j$(nproc)
    sudo make install -j$(nproc)
    cd /usr/bin
    err_conti_exec "sudo mv openssl openssl.old" "${dirname}" "${filename}"
    sudo bash -c "echo -e \"#!/bin/sh\nexport PATH=${libssl_dir}/bin:${PATH}\nexport LD_LIBRARY_PATH=${libssl_dir}/lib:${LD_LIBRARY_PATH}\" >> ${libssl_path_load_sh}"
    sudo chmod +x ${libssl_path_load_sh}
    source ${libssl_path_load_sh}
}

install_req () {
    echo_notice "${dirname}" "${filename}" "Installing required packages..."
    sudo apt $apt_gflag update
    sudo apt $apt_gflag upgrade -y
    #sudo apt $apt_gflag autoremove -y
    aptins "git"
    aptins "htop"
    #aptins "s-tui"
    #aptins "plocate"
    aptins "iftop"
    aptins "curl"
    aptins "wget"
    aptins "net-tools"
    aptins "neovim"
    aptins "build-essential"
    aptins "gcc"
    aptins "make"
    aptins "moreutils"
    aptins "tmux"
    if [ ${install_platform} -eq 1 ] || [ ${install_platform} -eq 4 ] || [ ${install_platform} -eq 5 ]; then
        aptins "libtool"
        aptins "autoconf"
        aptins "unzip"
    fi
    if [ ${install_platform} -eq 2 ] || [ ${install_platform} -eq 3 ] || [ ${install_platform} -eq 5 ]; then
        aptins "libssl-dev"
    fi
}

compile_req () {
    if [ ${install_platform} -eq 1 ] || [ ${install_platform} -eq 4 ]; then
        #build_libssl # This somehow breaks the system
        build_cmake
        cd $working_dir
    elif [ ${install_platform} -eq 5 ]; then
        build_cmake
        cd $working_dir
    fi
    build_valgrind
    cd $working_dir
}

config_nvim () {
    echo_notice "${dirname}" "${filename}" "Configuring neovim..."
    err_conti_exec "mkdir -p $nvim_dir" "common" "setup_config_nvim"
    wget $wget_gflag "$nvim_config_url" -O "${nvim_dir}/init.vim"
}

change_all_sh_mod () {
    echo_notice "${dirname}" "${filename}" "Changing all .sh files to executable..."
    find .. -type f -iname "*.sh" -exec sudo chmod +x {} \;
}

update_src () {
    echo_notice "${dirname}" "${filename}" "Pulling latest repo source..."
    git stash
    git stash clear
    err_retry_exec "git pull" 1 5 "common" "setup" 1
}

config_apport () {
    echo_notice "${dirname}" "${filename}" "Configuring apport..."
    ulimit -c unlimited
    err_conti_exec "mkdir -p $apport_dir" "common" "setup_config_apport"
    touch $apport_dir/settings
    echo -e "[main]\nunpackaged=true\n" > $apport_dir/settings
    rm -rf /var/crash/*
    sudo service whoopsie stop
    echo_notice "${dirname}" "${filename}" "Core dumps will be generated in /var/crash"
}

reload_term () {
    echo_notice "${dirname}" "${filename}" "Reloading terminal..."
    source ~/.bashrc
}

enable_ssh () {
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ]; then
        echo_notice "${dirname}" "${filename}" "Enabling ssh..."
        aptins "openssh-server"
        sudo systemctl enable ssh
        sudo systemctl start ssh
        sudo ufw allow ssh
    elif [ $install_platform -eq 2 ] || [ $install_platform -eq 3 ] || [ ${install_platform} -eq 5 ]; then
        echo_notice "${dirname}" "${filename}" "Enabling ssh..."
        sudo systemctl enable ssh
        sudo systemctl start ssh
    else
        echo_warn "${dirname}" "${filename}" "Invalid Argument: $install_platform ! Skipping enable_ssh..."
    fi
}

enable_pi_spi () {
    if [ $install_platform -eq 3 ]; then
        hardware_config="/boot/config.txt"
        echo_notice "${dirname}" "${filename}" "Enabling spi with ${hardware_config} ..."
        sudo sed -i 's/dtparam=spi=off/dtparam=spi=on/g' $hardware_config
        sudo sed -i 's/#dtparam=spi=on/dtparam=spi=on/g' $hardware_config
    elif [ $install_platform -eq 4 ]; then
        echo_notice "${dirname}" "${filename}" "SPI is one by default in this platform"
    elif [ $install_platform -eq 5 ]; then
        echo_notice "${dirname}" "${filename}" "SPI is skipped in this platform"
    else
        echo_warn "${dirname}" "${filename}" "Invalid Argument: $install_platform ! Skipping enable_pi_spi..."
    fi
}

echo_notice "${dirname}" "${filename}" "Current directory: $PWD"
working_dir=$PWD

if [ $job_update_src        -eq 1 ]; then update_src;        fi
if [ $job_change_all_sh_mod -eq 1 ]; then change_all_sh_mod; fi
if [ $job_enable_ssh        -eq 1 ]; then enable_ssh;        fi
if [ $job_enable_pi_spi     -eq 1 ]; then enable_pi_spi;     fi
if [ $job_config_nvim       -eq 1 ]; then config_nvim;       fi
if [ $job_config_apport     -eq 1 ]; then config_apport;     fi
if [ $job_install_req       -eq 1 ]; then install_req;       fi
if [ $job_compile_req       -eq 1 ]; then compile_req;       fi
if [ $job_reload_term       -eq 1 ]; then reload_term;       fi

if [ $job_setup_environment -eq 1 ]; then
    echo_warn "${dirname}" "${filename}" "Running environment setup Not Implemneted Yet!"
else
    echo_notice "${dirname}" "${filename}" "Skipped setup_environment"
fi

cd $working_dir
if [ $job_setup_ibmtpm -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running ibmtpm setup..."
    cd ../setup_ibmtpm
    sudo install_platform=$install_platform user=${USER} bash ./setup_sudo.sh
elif [ $job_setup_ibmtpm -eq 2 ]; then
    echo_notice "${dirname}" "${filename}" "Running ibmtpm setup..."
    cd ../setup_ibmtpm
    bash ./setup.sh
else
    echo_notice "${dirname}" "${filename}" "Skipped setup_ibmtpm"
fi

cd $working_dir
if [ $job_socket_com -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running socket_com setup..."
    cd ../socket_com
    install_platform=$install_platform bash ./setup.sh
else
    echo_notice "${dirname}" "${filename}" "Skipped setup_socket_com"
fi

cd $working_dir
if [ $job_setup_optiga -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running optiga setup..."
    cd ../setup_optiga
    install_platform=$install_platform bash ./setup.sh
else
    echo_notice "${dirname}" "${filename}" "Skipped setup_optiga"
fi

cd $working_dir
if [ $job_deploy_repo -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running deploy_repo setup..."
    cd ../deploy_repo
    install_platform=$install_platform bash ./setup.sh
else
    echo_notice "${dirname}" "${filename}" "Skipped setup_deploy_repo"
fi

cd $working_dir
if [ $job_update_swtpm -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running update_swtpm setup..."
    cd ../update_swtpm
    sudo install_platform=$install_platform bash ./setup_swtpm_isolated.sh
else
    echo_notice "${dirname}" "${filename}" "Skipped setup_swtpm"
fi

cd $working_dir
if [ $job_boot -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running boot setup..."
    cd ../boot
    install_platform=$install_platform bash ./setup_mbc_last.sh
else
    echo_notice "${dirname}" "${filename}" "Skipped setup_mbc_lst"
fi

cd $working_dir
if [ $job_acs_routine -eq 1 ]; then
    echo_notice "${dirname}" "${filename}" "Running acs_routine setup..."
    cd ../acs_routine
    install_platform=$install_platform bash ./setup_acsroutine.sh
else
    echo_notice "${dirname}" "${filename}" "Skipped setup_mbc_last"
fi

clear_preset

echo_notice "${dirname}" "${filename}" "All setup complete."
