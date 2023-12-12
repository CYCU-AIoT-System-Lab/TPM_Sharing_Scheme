#/bin/bash

user="user"
term_notice="\033[1m\033[34m[NOTICE-common/setup]\033[0m "
term_warn="\033[1m\033[33m[WARN-common/setup]\033[0m "
nvim_config_url="https://raw.githubusercontent.com/belongtothenight/config-files/main/ubuntu_init.vim"
nvim_dir="/home/${user}/.config/nvim"
apport_dir="/home/${user}/.config/apport"
cmake_ver="3.18"
cmake_build="4"
cmake_dir="/home/${user}/cmake-${cmake_ver}"
valgrind_ver="3.22.0"
valgrind_dir="/home/${user}/valgrind-${valgrind_ver}"

# sub_tasks (1=Enable)
install_for_pi=0
setup_environment=0 # Not implemented
setup_ibmtpm=0      # Not implemented
setup_socket_com=1

# Functions

build_cmake () {
	echo -e "${term_notice}Building cmake..."
	mkdir -p $cmake_dir
	cd $cmake_dir
	wget "https://cmake.org/files/v${cmake_ver}/cmake-${cmake_ver}.${cmake_build}.tar.gz"
	tar -xzvf "cmake-${cmake_ver}.${cmake_build}.tar.gz"
	cd "cmake-${cmake_ver}.${cmake_build}"
	./bootstrap
	make -j$(nproc)
	sudo make install
	cmake --version
}

build_valgrind () {
	echo -e "${term_notice}Building valgrind..."
	mkdir -p $valgrind_dir
	cd $valgrind_dir
	wget "https://sourceware.org/pub/valgrind/valgrind-${valgrind_ver}.tar.bz2"
	tar xvf "valgrind-${valgrind_ver}.tar.bz2"
	cd "valgrind-${valgrind_ver}"
	./configure
	make -j$(nproc)
	sudo make install
}

install_req () {
	aptins () {
		echo -e "${term_notice}Installing $1..."
		sudo apt-get install -y $1
	}
	echo -e "${term_notice} Installing required packages..."
	sudo apt-get update
	sudo apt-get upgrade -y
	aptins "git"
	aptins "htop"
	aptins "iftop"
	aptins "curl"
	aptins "wget"
	aptins "neovim"
	aptins "build-essential"
	aptins "gcc"
	aptins "make"
	if [ ${install_for_pi} -eq 0 ]; then
		aptins "libtool"
		aptins "autoconf"
		aptins "unzip"
		aptins "libssl-dev"
		build_cmake
		cd $working_dir
	fi
	build_valgrind
	cd $working_dir
}

config_nvim () {
	echo -e "${term_notice}Configuring neovim..."
	mkdir -p $nvim_dir
	wget "$nvim_config_url" -O "${nvim_dir}/init.vim"
}

change_all_sh_mod () {
	echo -e "${term_notice}Changing all .sh files to executable..."
	find .. -type f -iname "*.sh" -exec sudo chmod +x {} \;
}

update_src () {
	echo -e "${term_notice}Pulling latest repo source..."
	git stash
	git stash clear
	git pull
}

config_apport () {
	echo -e "${term_notice}Configuring apport..."
	ulimit -c unlimited
	mkdir -p $apport_dir
	touch $apport_dir/settings
	echo -e "[main]\nunpackaged=true\n" > $apport_dir/settings
	rm -rf /var/crash/*
	sudo service whoopsie stop
	echo -e "${term_notice}Core dumps will be generated in /var/crash"
}

reload_term () {
	echo -e "${term_notice}Reloading terminal..."
	source ~/.bashrc
}

echo -e "${term_notice}Running common setup..."
echo -e "${term_notice}Current directory: $PWD"
working_dir=$PWD
install_req
config_nvim
update_src
change_all_sh_mod
config_apport
reload_term
echo -e "${term_notice}Common setup complete."

if [ $setup_environment -eq 1 ]; then
	echo -e "${term_warn}Running environment setup Not Implemneted Yet!"
else
	echo -e "${term_warn}Invalid Argument: $setup_environment ! Skipping setup_environment..."
fi

if [ $setup_ibmtpm -eq 1 ]; then
	echo -e "${term_warn}Running ibmtpm setup Not Implemneted Yet!"
else
	echo -e "${term_warn}Invalid Argument: $setup_ibmtpm ! Skipping setup_ibmtpm..."
fi

cd $working_dir
if [ $setup_socket_com -eq 1 ]; then
	echo -e "${term_notice}Running socket_com setup..."
	cd ../socket_com
	./setup.sh
	cd ..
else
	echo -e "${term_warn}Invalid Argument: $setup_socket_com ! Skipping setup_socket_com..."
fi

echo -e "${term_notice}All setup complete."
