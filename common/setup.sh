#/bin/bash

term_notice="\033[1m\033[34m[NOTICE-common/setup]\033[0m "
term_warn="\033[1m\033[33m[WARN-common/setup]\033[0m "
nvim_config_url="https://github.com/belongtothenight/config-files/blob/main/ubuntu_init.vim"
nvim_dir="/home/user/.config/nvim"
repo_url="git@github.com:CYCU-AIoT-System-Lab/TPM_Sharing_Scheme.git"

# sub_tasks (1=Enable)
setup_environment=0 # Not implemented
setup_ibmtmp=0      # Not implemented
setup_socket_com=1

install_req () {
	aptins () {
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
}

config_nvim () {
	echo -e "$term_notice Configuring neovim..."
	mkdir -p $nvim_dir
	wget $nvim_config_url -O "${nvim_dir}/init.vim"
}

change_all_sh_mod () {
	echo -e "$term_notice Changing all .sh files to executable..."
	chmod +x *.sh
	chmod +x setup_environment/*.sh
	chmod +x setup_ibmtmp/*.sh
	chmod +x socket_com/*.sh
}

update_src () {
	echo -e "$term_notice Pulling latest repo source..."
	git stash
	git stash clear
	git pull
}

reload_term () {
	echo -e "$term_notice Reloading terminal..."
	reset
}

echo -e "$term_notice Running common setup..."
install_req
config_nvim
change_all_sh_mod
update_src
reload_term
echo -e "$term_notice Common setup complete."

if [ $setup_environment -eq 1 ]; then
	echo -e "$term_warn Running environment setup Not Implemneted Yet!"
else
	echo -e "$term_warn Invalid Argument: $setup_environment ! Skipping..."
fi

if [ $setup_ibmtmp -eq 1 ]; then
	echo -e "$term_warn Running ibmtmp setup Not Implemneted Yet!"
else
	echo -e "$term_warn Invalid Argument: $setup_ibmtmp ! Skipping..."
fi

if [ $setup_socket_com -eq 1 ]; then
	echo -e "$term_notice Running socket_com setup..."
	cd ../socket_com
	./setup.sh
	cd ..
else
	echo -e "$term_warn Invalid Argument: $setup_socket_com ! Skipping..."
fi

echo -e "$term_notice All setup complete."
