#!/bin/bash

# Setup Tasks (0=No, 1=Yes)
run_client=1
run_server=1

# General Settings
proj_dir="${PWD}"
term_notice="\033[1m\033[34m[NOTICE]\033[0m "
term_warn="\033[1m\033[33m[WARNING]\033[0m "

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
else
	echo -e "${term_warn}Client compiling task disabled."
fi

# Add server subproject
if [ $compile_server -eq 1 ]; then
	echo -e "${term_notice}Adding server compiling task..."
	echo "add_subdirectory(\${CMAKE_SOURCE_DIR}/server)" >> "${proj_dir}/CMakeLists.txt"
else
	echo -e "${term_warn}Server compiling task disabled."
fi


# Finish
echo -e "${term_notice}Setup complete."
