#!/bin/bash

# General Settings
proj_dir="${PWD}"
conf_file="${proj_dir}/config.ini"
term_notice="\033[1m\033[34m[NOTICE]\033[0m "
term_warn="\033[1m\033[33m[WARNING]\033[0m "

# Do not adjust below this line
echo -e "${term_notice}Running setup script..."


# Read config.ini
# -----------------------------

# Setup Tasks (0=No, 1=Yes)
run_client=$(awk -F "=" '/^run_client/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
run_server=$(awk -F "=" '/^run_server/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")

# SubTasks-dev (0=No, 1=Yes)
perform_clean=$(awk -F "=" '/^perform_clean/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
perform_build=$(awk -F "=" '/^perform_build/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")

# Option Conditioning
# -----------------------------
compile_client=$((perform_build * run_client))
compile_server=$((perform_build * run_server))

# Show Options
# -----------------------------
echo -e "${term_notice}Settings in ${conf_file}:"
echo -e "run_client:     ${run_client}"
echo -e "run_server:     ${run_server}"
echo -e "perform_clean:  ${perform_clean}"
echo -e "perform_build:  ${perform_build}"
echo -e "compile_client: ${compile_client}"
echo -e "compile_server: ${compile_server}"

# Main Flow
# -----------------------------

#! Clean
cd "${proj_dir}/build"
if [ $perform_clean -eq 1 ]; then
	echo -e "${term_notice}Cleaning project..."
	rm -rf "${proj_dir}/build"
elif [ $perform_clean -eq 0 ]; then
	echo -e "${term_notice}Skipped cleaning project!"
else
	echo -e "${term_warn}Invalid Argument! Skipped cleaning project!"
fi


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
