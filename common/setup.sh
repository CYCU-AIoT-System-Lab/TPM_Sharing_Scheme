#/bin/bash

# Replace all ${HOME} with $HOME
sed -i 's@${HOME}@'"$HOME"'@' config.ini

source "./function.sh"
parse "./config.ini" "display"

# Functions

build_cmake () {
    echo_notice "common" "setup" "Building ${BOLD}${GREEN}cmake${END}..."
	mkdir -p $cmake_dir
	cd $cmake_dir
	wget $wget_gflag "https://cmake.org/files/v${cmake_ver}/cmake-${cmake_ver}.${cmake_build}.tar.gz"
	tar $tar_gflag "cmake-${cmake_ver}.${cmake_build}.tar.gz"
	cd "cmake-${cmake_ver}.${cmake_build}"
	./bootstrap
	make $make_gflag -j$(nproc)
	sudo $sudo_gflag make $make_gflag install
	cmake --version
}

build_valgrind () {
    echo_notice "common" "setup" "${BOLD}${GREEN}Building valgrind${END}..."
	mkdir -p $valgrind_dir
	cd $valgrind_dir
	wget $wget_gflag "https://sourceware.org/pub/valgrind/valgrind-${valgrind_ver}.tar.bz2"
	tar $tar_gflag "valgrind-${valgrind_ver}.tar.bz2"
	cd "valgrind-${valgrind_ver}"
	./configure
	make $make_gflag -j$(nproc)
	sudo $sudo_gflag make $make_gflag install
}

install_req () {
	aptins () {
        echo_notice "common" "setup" "Installing ${BOLD}${GREEN}$1${END}..."
		sudo apt-get $apt_gflag install $1 -y
	}
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
	aptins "libssl-dev"
	if [ ${install_for_pi} -eq 0 ]; then
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

echo_notice "common" "setup" "Running common setup..."
echo_notice "common" "setup" "Current directory: $PWD"
working_dir=$PWD
if [ $setup_mutual_req -eq 1 ]; then
    echo_notice "common" "setup" "Running mutual setup..."
    install_req
    config_nvim
    update_src
    change_all_sh_mod
    config_apport
    reload_term
else
    echo_warn "common" "setup" "Invalid Argument: $setup_mutual_req ! Skipping mutual setup..."
fi

if [ $setup_environment -eq 1 ]; then
    echo_warn "common" "setup" "Running environment setup Not Implemneted Yet!"
else
    echo_warn "common" "setup" "Invalid Argument: $setup_environment ! Skipping setup_environment..."
fi

cd $working_dir
if [ $setup_ibmtpm -eq 1 ]; then
    echo_notice "common" "setup" "Running ibmtpm setup..."
    cd ../setup_ibmtpm
    sudo bash ./setup_sudo.sh
elif [ $setup_ibmtpm -eq 2 ]; then
    echo_notice "common" "setup" "Running ibmtpm setup..."
    cd ../setup_ibmtpm
    bash ./setup.sh
else
    echo_warn "common" "setup" "Invalid Argument: $setup_ibmtpm ! Skipping setup_ibmtpm..."
fi

cd $working_dir
if [ $setup_socket_com -eq 1 ]; then
    echo_notice "common" "setup" "Running socket_com setup..."
	cd ../socket_com
	bash ./setup.sh
else
    echo_warn "common" "setup" "Invalid Argument: $setup_socket_com ! Skipping setup_socket_com..."
fi

echo_notice "common" "setup" "All setup complete."
