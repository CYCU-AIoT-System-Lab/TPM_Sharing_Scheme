#!/bin/bash

# Setup Tasks (0=No, 1=Yes)
install_client=1
install_server=1

# SubTasks-dev (0=No, 1=Yes)

# General Settings
proj_dir="${PWD}"
term_notice="\033[1m\033[34m[NOTICE]\033[0m "
term_warn="\033[1m\033[33m[WARNING]\033[0m "

# Delete cmake subproject
awk '!/add_subdirectory/' "${proj_dir}/CMakeLists.txt" > "${proj_dir}/tmp.txt"
mv "${proj_dir}/tmp.txt" "${proj_dir}/CMakeLists.txt"

# Add client subproject
if [ $install_client -eq 1 ]; then
	echo -e "${term_notice}Adding client compiling task..."
	echo "add_subdirectory(${CMAKE_SOURCE_DIR}/client)" >> "${proj_dir}/CMakeLists.txt"
fi

# Add server subproject
if [ $install_server -eq 1 ]; then
	echo -e "${term_notice}Adding server compiling task..."
	echo "add_subdirectory(${CMAKE_SOURCE_DIR}/server)" >> "${proj_dir}/CMakeLists.txt"
fi


# Finish
echo -e "${term_notice}Setup complete."
