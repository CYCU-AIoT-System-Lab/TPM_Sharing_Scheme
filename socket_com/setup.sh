#!/bin/bash
#!/bin/bash -x # for debugging

# General Settings
proj_dir="${PWD}"
conf_file="${proj_dir}/config.ini"
term_notice_setup="\033[1m\033[34m[NOTICE-setup]\033[0m "
term_warn_setup="\033[1m\033[33m[WARNING-setup]\033[0m "
term_notice_server="\033[1m\033[34m[NOTICE-server]\033[0m "
term_warn_server="\033[1m\033[33m[WARNING-server]\033[0m "
term_notice_client="\033[1m\033[34m[NOTICE-client]\033[0m "
term_warn_client="\033[1m\033[33m[WARNING-client]\033[0m "
term_notice_docs="\033[1m\033[34m[NOTICE-docs]\033[0m "
term_warn_docs="\033[1m\033[33m[WARNING-docs]\033[0m "

# Do not adjust below this line
echo -e "${term_notice_setup}Running setup script..."

# Read config.ini
# -----------------------------

# Setup Tasks (0=No, 1=Yes)
run_server=$(awk -F "=" '/^run_server/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
run_client=$(awk -F "=" '/^run_client/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")

# SubTasks-dev (0=No, 1=Yes)
install_dependencies=$(awk -F "=" '/^install_dependencies/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
perform_clean=$(awk -F "=" '/^perform_clean/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
perform_build=$(awk -F "=" '/^perform_build/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
build_for_debug=$(awk -F "=" '/^build_for_debug/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
generate_docs=$(awk -F "=" '/^generate_docs/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
host_docs=$(awk -F "=" '/^host_docs/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
host_ip=$(awk -F "=" '/^host_ip/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
host_port=$(awk -F "=" '/^host_port/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
open_docs_in_browser=$(awk -F "=" '/^open_docs_in_browser/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")

# Option Conditioning
# -----------------------------
compile_server=$((perform_build * run_server))
compile_client=$((perform_build * run_client))
build_for_debug=$((perform_build * build_for_debug))

# Show Options
# -----------------------------
echo -e "${term_notice_setup}Settings in ${conf_file}:"
echo -e "run_server:           ${run_server}"
echo -e "run_client:           ${run_client}"
echo -e "install_dependencies: ${install_dependencies}"
echo -e "perform_clean:        ${perform_clean}"
echo -e "perform_build:        ${perform_build}"
echo -e "build_for_debug:      ${build_for_debug}"
echo -e "generate_docs:        ${generate_docs}"
echo -e "host_docs:            ${host_docs}"
echo -e "host_ip:              ${host_ip}"
echo -e "host_port:            ${host_port}"
echo -e "open_docs_in_browser: ${open_docs_in_browser}"
echo -e "compile_server:       ${compile_server}"
echo -e "compile_client:       ${compile_client}"

# Main Flow
# -----------------------------

#! Setup Directories
build_dir="${proj_dir}/build"
bin_dir="${proj_dir}/bin"
doc_dir="${proj_dir}/doc"
echo -e "${term_notice_setup}Creating directories: ${build_dir}, ${bin_dir}"
mkdir $build_dir
mkdir $bin_dir

#! Install Dependencies
if [ $install_dependencies -eq 1 ]; then
	echo -e "${term_notice_setup}Installing dependencies..."
	sudo apt-get update
	sudo apt-get update -y --fix-missing
	sudo apt-get install -y neovim cmake build-essential doxygen graphviz
elif [ $install_dependencies -eq 0 ]; then
	echo -e "${term_notice_setup}Skipped installing dependencies!"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped installing dependencies!"
fi

#! Clean
cd "${proj_dir}/build"
if [ $perform_clean -eq 1 ]; then
	echo -e "${term_notice_setup}Cleaning project..."
	rm -rf "${proj_dir}/build/*"
elif [ $perform_clean -eq 0 ]; then
	echo -e "${term_notice_setup}Skipped cleaning project!"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped cleaning project!"
fi

#! Build
cd "${proj_dir}/build"

# Delete cmake subproject
awk '!/add_subdirectory/' "${proj_dir}/CMakeLists.txt" > "${proj_dir}/tmp.txt"
mv "${proj_dir}/tmp.txt" "${proj_dir}/CMakeLists.txt"

# Add server subproject
if [ $compile_server -eq 1 ]; then
	echo -e "${term_notice_setup}Adding server compiling task..."
	echo "add_subdirectory(\${CMAKE_SOURCE_DIR}/server)" >> "${proj_dir}/CMakeLists.txt"
elif [ $compile_server -eq 0 ]; then
	echo -e "${term_notice_setup}Skipped server compiling task!"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped server compiling task!"
fi

# Add client subproject
if [ $compile_client -eq 1 ]; then
	echo -e "${term_notice_setup}Adding client compiling task..."
	echo "add_subdirectory(\${CMAKE_SOURCE_DIR}/client)" >> "${proj_dir}/CMakeLists.txt"
elif [ $compile_client -eq 0 ]; then
	echo -e "${term_notice_setup}Skipped client compiling task!"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped client compiling task!"
fi

# Build
if [ $build_for_debug -eq 1 ]; then
	echo -e "${term_notice_setup}Generating makefile for debug..."
	cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=ON
	echo -e "${term_notice_setup}Building and installing project..."
	make -j"$(nproc)" install
elif [ $build_for_debug -eq 0 ]; then
	echo -e "${term_notice_setup}Generating makefile for release..."
	cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=ON
	echo -e "${term_notice_setup}Building and installing project..."
	make -j"$(nproc)" install
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped generating makefile!"
fi

#! Run

# Run Client
cd "${proj_dir}/bin"

# Run Server
cd "${proj_dir}/bin"
if [ $run_server -eq 1 ]; then
	echo -e "${term_notice_setup}Running server on new terminal..."
	launch_cmd="echo -e \"${term_notice_server}Starting server...\"; ./server"
	gnome-terminal -- bash -c "${launch_cmd}; exec bash"
elif [ $run_server -eq 0 ]; then
	echo -e "${term_notice_setup}Skipped running server!"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped running server!"
fi

# Run Client
if [ $run_client -eq 1 ]; then
	echo -e "${term_notice_setup}Running client on new terminal..."
	launch_cmd="echo -e \"${term_notice_client}Starting client...\"; ./client"
	gnome-terminal -- bash -c "${launch_cmd}; exec bash"
elif [ $run_client -eq 0 ]; then
	echo -e "${term_notice_setup}Skipped running client!"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped running client!"
fi

#! Generate Documentation
cd "${proj_dir}"
if [ $generate_docs -eq 1 ]; then
	echo -e "${term_notice_setup}Generating documentation..."
	doxygen ./Doxyfile
elif [ $generate_docs -eq 0 ]; then
	echo -e "${term_notice_setup}Skipped generating documentation!"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped generating documentation!"
fi

#! Host Documentation
if [ $host_docs -eq 1 ]; then
	echo -e "${term_notice_setup}Hosting documentation in new terminal..."
	cd "${doc_dir}/html"
	launch_cmd="echo -e \"${term_notice_docs}Hosting documentation...\"; python3 -m http.server ${host_port} --bind ${host_ip} --directory ${doc_dir}/html"
	gnome-terminal -- bash -c "${launch_cmd}; exec bash"
elif [ $host_docs -eq 0 ]; then
	echo -e "${term_notice_setup}Skipped hosting documentation!"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped hosting documentation!"
fi

# Finish
cd "${proj_dir}"
echo -e "${term_notice_setup}Setup complete."
