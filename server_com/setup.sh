#!/bin/bash

# General Settings
proj_dir="${PWD}"
term_notice="\033[1m\033[34m[NOTICE]\033[0m "
term_warn="\033[1m\033[33m[WARNING]\033[0m "

# Setup Tasks (0=No, 1=Yes)
run_client=$(gawk -F "=" "/run_client/ {print $2}" "${proj_dir}/config.ini")
run_server=1

# SubTasks-dev (0=No, 1=Yes)
perform_clean=1
perform_build=1

# Do not adjust below this line
# -----------------------------
compile_client=$((perform_build * run_client))
compile_server=$((perform_build * run_server))

# Delete cmake subproject
awk '!/add_subdirectory/' "${proj_dir}/CMakeLists.txt" > "${proj_dir}/tmp.txt"
mv "${proj_dir}/tmp.txt" "${proj_dir}/CMakeLists.txt"

# Add client subproject
if [ $compile_client -eq 1 ]; then
	echo -e "${term_notice}Adding client compiling task..."
	echo "add_subdirectory(\${CMAKE_SOURCE_DIR}/client)" >> "${proj_dir}/CMakeLists.txt"
elif [ $compile_client -eq 0 ]; then
	echo -e "${term_notice}Skipped client compiling task!"
else
	echo -e "${term_warn}Invalid Argument! Skipped client compiling task!"
fi

# Add server subproject
if [ $compile_server -eq 1 ]; then
	echo -e "${term_notice}Adding server compiling task..."
	echo "add_subdirectory(\${CMAKE_SOURCE_DIR}/server)" >> "${proj_dir}/CMakeLists.txt"
elif [ $compile_server -eq 0 ]; then
	echo -e "${term_notice}Skipped server compiling task!"
else
	echo -e "${term_warn}Invalid Argument! Skipped server compiling task!"
fi


# Finish
echo -e "${term_notice}Setup complete."
