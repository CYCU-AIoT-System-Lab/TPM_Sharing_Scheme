#/bin/bash

source "./function.sh"
source "./function_common.sh"
load_preset "./config.ini"

# Functions

build_cmake () {
    echo_notice "common" "setup" "Building ${BOLD}${GREEN}cmake${END}..."
	mkdir -p $cmake_dir
	cd $cmake_dir
	wget $wget_gflag "https://cmake.org/files/v${cmake_ver}/cmake-${cmake_ver}.${cmake_build}.tar.gz"
	tar -zxf "cmake-${cmake_ver}.${cmake_build}.tar.gz"
	cd "cmake-${cmake_ver}.${cmake_build}"
	./bootstrap
	make $make_gflag -j$(nproc)
	sudo $sudo_gflag make $make_gflag install
	cmake --version
}

build_valgrind () {
    echo_notice "common" "setup" "Building ${BOLD}${GREEN}valgrind${END}..."
	mkdir -p $valgrind_dir
	cd $valgrind_dir
	wget $wget_gflag "https://sourceware.org/pub/valgrind/valgrind-${valgrind_ver}.tar.bz2"
	tar xfj "valgrind-${valgrind_ver}.tar.bz2"
	cd "valgrind-${valgrind_ver}"
	./configure
	make $make_gflag -j$(nproc)
	sudo $sudo_gflag make $make_gflag install
}

install_req () {
    echo_notice "common" "setup" "Installing required packages..."
	sudo apt-get $apt_gflag update
	sudo apt-get $apt_gflag upgrade -y
	aptins "git"
	aptins "htop"
	aptins "iftop"
	aptins "curl"
	aptins "wget"
	aptins "net-tools"
	aptins "neovim"
	aptins "build-essential"
	aptins "gcc"
	aptins "make"
	#aptins "libssl-dev" # Version 
    aptins "moreutils"
	if [ ${install_platform} -eq 1 ] || [ ${install_platform} -eq 4 ]; then
		aptins "libtool"
		aptins "autoconf"
		aptins "unzip"
		build_cmake
		cd $working_dir
	fi
	build_valgrind
	cd $working_dir
}

config_nvim () {
    echo_notice "common" "setup" "Configuring neovim..."
	mkdir -p $nvim_dir
	wget $wget_gflag "$nvim_config_url" -O "${nvim_dir}/init.vim"
}

change_all_sh_mod () {
    echo_notice "common" "setup" "Changing all .sh files to executable..."
	find .. -type f -iname "*.sh" -exec sudo chmod +x {} \;
}

update_src () {
    echo_notice "common" "setup" "Pulling latest repo source..."
	git stash
	git stash clear
	git pull
}

config_apport () {
    echo_notice "common" "setup" "Configuring apport..."
	ulimit -c unlimited
	mkdir -p $apport_dir
	touch $apport_dir/settings
	echo -e "[main]\nunpackaged=true\n" > $apport_dir/settings
	rm -rf /var/crash/*
	sudo service whoopsie stop
    echo_notice "common" "setup" "Core dumps will be generated in /var/crash"
}

reload_term () {
    echo_notice "common" "setup" "Reloading terminal..."
	source ~/.bashrc
}

enable_ssh () {
    if [ $install_platform -eq 1 ] || [ $install_platform -eq 4 ]; then
        echo_notice "common" "setup" "Enabling ssh..."
        aptins "openssh-server"
        sudo systemctl enable ssh
        sudo systemctl start ssh
        sudo ufw allow ssh
    elif [ $install_platform -eq 2 ] || [ $install_platform -eq 3 ]; then
        echo_notice "common" "setup" "Enabling ssh..."
        sudo systemctl enable ssh
        sudo systemctl start ssh
    else
        echo_warn "common" "setup" "Invalid Argument: $install_platform ! Skipping enable_ssh..."
    fi
}

enable_pi_spi () {
    if [ $install_platform -eq 3 ]; then
        hardware_config="/boot/config.txt"
        echo_notice "common" "setup" "Enabling spi with ${hardware_config} ..."
        sudo sed -i 's/dtparam=spi=off/dtparam=spi=on/g' $hardware_config
        sudo sed -i 's/#dtparam=spi=on/dtparam=spi=on/g' $hardware_config
    elif [ $install_platform -eq 4 ]; then
        echo_notice "common" "setup" "SPI is one by default in this platform"
    else
        echo_warn "common" "setup" "Invalid Argument: $install_platform ! Skipping enable_pi_spi..."
    fi
}

echo_notice "common" "setup" "Running common setup..."
echo_notice "common" "setup" "Current directory: $PWD"
working_dir=$PWD

if [ $job_enable_ssh        -eq 1 ]; then enable_ssh;        fi
if [ $job_enable_pi_spi     -eq 1 ]; then enable_pi_spi;     fi
if [ $job_install_req       -eq 1 ]; then install_req;       fi
if [ $job_config_nvim       -eq 1 ]; then config_nvim;       fi
if [ $job_update_src        -eq 1 ]; then update_src;        fi
if [ $job_change_all_sh_mod -eq 1 ]; then change_all_sh_mod; fi
if [ $job_config_apport     -eq 1 ]; then config_apport;     fi
if [ $job_reload_term       -eq 1 ]; then reload_term;       fi

if [ $job_setup_environment -eq 1 ]; then
    echo_warn "common" "setup" "Running environment setup Not Implemneted Yet!"
else
    echo_warn "common" "setup" "Invalid Argument: $job_setup_environment ! Skipping setup_environment..."
fi

cd $working_dir
if [ $job_setup_ibmtpm -eq 1 ]; then
    echo_notice "common" "setup" "Running ibmtpm setup..."
    cd ../setup_ibmtpm
    sudo install_platform=$install_platform bash ./setup_sudo.sh
elif [ $job_setup_ibmtpm -eq 2 ]; then
    echo_notice "common" "setup" "Running ibmtpm setup..."
    cd ../setup_ibmtpm
    bash ./setup.sh
else
    echo_warn "common" "setup" "Invalid Argument: $job_setup_ibmtpm ! Skipping setup_ibmtpm..."
fi

cd $working_dir
if [ $job_socket_com -eq 1 ]; then
    echo_notice "common" "setup" "Running socket_com setup..."
	cd ../socket_com
	install_platform=$install_platform bash ./setup.sh
else
    echo_warn "common" "setup" "Invalid Argument: $job_socket_com ! Skipping setup_socket_com..."
fi

cd $working_dir
if [ $job_setup_optiga -eq 1 ]; then
    echo_notice "common" "setup" "Running optiga setup..."
    cd ../setup_optiga
    install_platform=$install_platform bash ./setup.sh
else
    echo_warn "common" "setup" "Invalid Argument: $job_setup_optiga ! Skipping setup_optiga..."
fi

clear_preset

echo_notice "common" "setup" "All setup complete."
