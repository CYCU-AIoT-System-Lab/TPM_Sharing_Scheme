#/bin/bash

user="user"
term_notice="\033[1m\033[34m[NOTICE-common/setup]\033[0m "
term_warn="\033[1m\033[33m[WARN-common/setup]\033[0m "
nvim_config_url="https://raw.githubusercontent.com/belongtothenight/config-files/main/ubuntu_init.vim"
nvim_dir="/home/${user}/.config/nvim"
apport_dir="/home/${user}/.config/apport"

# sub_tasks (1=Enable)
setup_environment=0 # Not implemented
setup_ibmtpm=0      # Not implemented
setup_socket_com=1

install_req () {
	aptins () {
		echo -e "$term_notice Installing $1..."
		sudo apt-get install -y $1
	}
	echo -e "$term_notice Installing required packages..."
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
	aptins "cmake"
	aptins "valgrind"
}

config_nvim () {
	echo -e "$term_notice Configuring neovim..."
	mkdir -p $nvim_dir
	wget "$nvim_config_url" -O "${nvim_dir}/init.vim"
}

change_all_sh_mod () {
	echo -e "$term_notice Changing all .sh files to executable..."
	find .. -type f -iname "*.sh" -exec sudo chmod +x {} \;
}

update_src () {
	echo -e "$term_notice Pulling latest repo source..."
	git stash
	git stash clear
	git pull
}

config_apport () {
	echo -e "$term_notice Configuring apport..."
	ulimit -c unlimited
	mkdir -p $apport_dir
	touch $apport_dir/settings
	echo -e "[main]\nunpackaged=true\n" > $apport_dir/settings
	rm -rf /var/crash/*
	sudo service whoopsie stop
	echo -e "${term_notice_setup}Core dumps will be generated in /var/crash"
}

reload_term () {
	echo -e "$term_notice Reloading terminal..."
	source ~/.bashrc
}

echo -e "$term_notice Running common setup..."
echo -e "$term_notice Current directory: $PWD"
install_req
config_nvim
update_src
change_all_sh_mod
config_apport
reload_term
echo -e "$term_notice Common setup complete."

if [ $setup_environment -eq 1 ]; then
	echo -e "$term_warn Running environment setup Not Implemneted Yet!"
else
	echo -e "$term_warn Invalid Argument: $setup_environment ! Skipping setup_environment..."
fi

if [ $setup_ibmtpm -eq 1 ]; then
	echo -e "$term_warn Running ibmtpm setup Not Implemneted Yet!"
else
	echo -e "$term_warn Invalid Argument: $setup_ibmtpm ! Skipping setup_ibmtpm..."
fi

if [ $setup_socket_com -eq 1 ]; then
	echo -e "$term_notice Running socket_com setup..."
	cd ../socket_com
	./setup.sh
	cd ..
else
	echo -e "$term_warn Invalid Argument: $setup_socket_com ! Skipping setup_socket_com..."
fi

echo -e "$term_notice All setup complete."
