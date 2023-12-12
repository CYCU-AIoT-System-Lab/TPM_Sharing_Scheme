#!/bin/bash
#!/bin/bash -x # for debugging

# General Settings
proj_dir="${PWD}"
conf_file="${proj_dir}/config.ini"
term_notice_setup="\033[1m\033[34m[NOTICE-socket_com/setup]\033[0m "
term_warn_setup="\033[1m\033[33m[WARN-socket_com/setup]\033[0m "
term_notice_server="\033[1m\033[34m[NOTICE-socket_com/server]\033[0m "
term_warn_server="\033[1m\033[33m[WARN-socket_com/server]\033[0m "
term_notice_client="\033[1m\033[34m[NOTICE-socket_com/client]\033[0m "
term_warn_client="\033[1m\033[33m[WARN-socket_com/client]\033[0m "
term_notice_docs="\033[1m\033[34m[NOTICE-socket_com/docs]\033[0m "
term_warn_docs="\033[1m\033[33m[WARN-socket_com/docs]\033[0m "

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
build_project_name=$(awk -F "=" '/^build_project_name/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
build_project_version=$(awk -F "=" '/^build_project_version/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
check_tool=$(awk -F "=" '/^check_tool/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
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
echo -e "run_server:             ${run_server}"
echo -e "run_client:             ${run_client}"
echo -e "install_dependencies:   ${install_dependencies}"
echo -e "perform_clean:          ${perform_clean}"
echo -e "perform_build:          ${perform_build}"
echo -e "build_for_debug:        ${build_for_debug}"
echo -e "build_project_name:     ${build_project_name}"
echo -e "build_project_version:  ${build_project_version}"
echo -e "check_tool:             ${check_tool}"
echo -e "generate_docs:          ${generate_docs}"
echo -e "host_docs:              ${host_docs}"
echo -e "host_ip:                ${host_ip}"
echo -e "host_port:              ${host_port}"
echo -e "open_docs_in_browser:   ${open_docs_in_browser}"
echo -e "compile_server:         ${compile_server}"
echo -e "compile_client:         ${compile_client}"

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
aptins () {
	echo -e "${term_notice_setup}Installing $1..."
	sudo apt-get install -y $1
}
if [ $install_dependencies -eq 1 ]; then
	echo -e "${term_notice_setup}Installing dependencies..."
	sudo apt-get update
	sudo apt-get update -y --fix-missing
	aptins "doxygen"
	aptins "graphviz"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped installing dependencies!"
fi

#! Clean
cd "${proj_dir}/build"
if [ $perform_clean -eq 1 ]; then
	echo -e "${term_notice_setup}Cleaning project..."
	rm -rf "${proj_dir}/build/*"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped cleaning project!"
fi

#! Build
cd "${proj_dir}/build"

# Delete cmake presets
echo -e "${term_notice_setup}Deleting cmake presets..."
awk '!/set\(PROJECT_NAME/' "${proj_dir}/CMakeLists.txt" > "${proj_dir}/tmp.txt"
mv "${proj_dir}/tmp.txt" "${proj_dir}/CMakeLists.txt"
awk '!/project\(\${PROJECT_NAME} VERSION/' "${proj_dir}/CMakeLists.txt" > "${proj_dir}/tmp.txt"
mv "${proj_dir}/tmp.txt" "${proj_dir}/CMakeLists.txt"
awk '!/add_subdirectory/' "${proj_dir}/CMakeLists.txt" > "${proj_dir}/tmp.txt"
mv "${proj_dir}/tmp.txt" "${proj_dir}/CMakeLists.txt"

# Add cmake presets
echo -e "${term_notice_setup}Adding cmake presets..."
echo "set(PROJECT_NAME \"${build_project_name}\")" >> "${proj_dir}/CMakeLists.txt"
echo "project(\${PROJECT_NAME} VERSION ${build_project_version} LANGUAGES C)" >> "${proj_dir}/CMakeLists.txt"

# Replace cmake presets
if [ ${check_tool} = "ASAN" ]; then
	echo -e "${term_notice_setup}Replacing cmake presets..."
	sed -i -e 's/\"\"\/\"-fsanitize=address\"\/' "${proj_dir}/CMakeLists.txt"
fi

# Add server subproject
if [ $compile_server -eq 1 ]; then
	echo -e "${term_notice_setup}Adding server compiling task..."
	echo "add_subdirectory(\${CMAKE_SOURCE_DIR}/server)" >> "${proj_dir}/CMakeLists.txt"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped server compiling task!"
fi

# Add client subproject
if [ $compile_client -eq 1 ]; then
	echo -e "${term_notice_setup}Adding client compiling task..."
	echo "add_subdirectory(\${CMAKE_SOURCE_DIR}/client)" >> "${proj_dir}/CMakeLists.txt"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped client compiling task!"
fi

# Build
if [ $build_for_debug -eq 1 ]; then
	echo -e "${term_notice_setup}Generating makefile for debug..."
	cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=ON
	echo -e "${term_notice_setup}Building and installing project..."
	make -j"$(nproc)" install
else
	echo -e "${term_notice_setup}Generating makefile for release..."
	cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=ON
	echo -e "${term_notice_setup}Building and installing project..."
	make -j"$(nproc)" install
fi

#! Run

#  Check for memory leaks and Run Server
cd "${proj_dir}/bin"
if [ $run_server -eq 1 ]; then
	echo -e "${term_notice_setup}Running server on new terminal..."
	if [ ${check_tool} = "ASAN" ]; then
		launch_cmd1="echo -e \"${term_notice_server}Address Sanitizing...\""
		launch_cmd2="./server"
		launch_cmd3="echo -e \"${term_notice_server}Memory checked with Address Sanitizer.\""
		launch_cmd="${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}"
	elif [ ${check_tool} = "Valgrind" ]; then
		launch_cmd1="echo -e \"${term_notice_server}Memory Leak Checking (valgrind)...\""
		launch_cmd2="valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes -v ./server"
		launch_cmd3="echo -e \"${term_notice_server}Memory checked with Valgrind.\""
		launch_cmd="${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}"
	else
		launch_cmd="echo -e \"${term_notice_server}Starting server...\"; ./server"
	fi
	gnome-terminal -- bash -c "${launch_cmd}; exec bash"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped running server!"
fi

# Check for memory leaks and Run Client
if [ $run_client -eq 1 ]; then
	echo -e "${term_notice_setup}Running client on new terminal..."
	if [ ${check_tool} = "ASAN" ]; then
		launch_cmd1="echo -e \"${term_notice_client}Address Sanitizing...\""
		launch_cmd2="./client"
		launch_cmd3="echo -e \"${term_notice_client}Memory checked with Address Sanitizer.\""
		launch_cmd="${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}"
	elif [ ${check_tool} = "Valgrind" ]; then
		launch_cmd1="echo -e \"${term_notice_client}Memory Leak Checking (valgrind)...\""
		launch_cmd2="valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes -v ./client"
		launch_cmd3="echo -e \"${term_notice_client}Memory checked with Valgrind.\""
		launch_cmd="${launch_cmd1}; ${launch_cmd2}; ${launch_cmd3}"
	else
		launch_cmd="echo -e \"${term_notice_client}Starting client...\"; ./client"
	fi
	gnome-terminal -- bash -c "${launch_cmd}; exec bash"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped running client!"
fi

#! Generate Documentation
cd "${proj_dir}"
if [ $generate_docs -eq 1 ]; then
	echo -e "${term_notice_setup}Generating documentation..."
	doxygen ./Doxyfile
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped generating documentation!"
fi

#! Host Documentation
if [ $host_docs -eq 1 ]; then
	echo -e "${term_notice_setup}Hosting documentation in new terminal..."
	cd "${doc_dir}/html"
	launch_cmd="echo -e \"${term_notice_docs}Hosting documentation...\"; python3 -m http.server --bind ${host_ip} ${host_port}"
	gnome-terminal -- bash -c "${launch_cmd}; exec bash"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped hosting documentation!"
fi

#! Open Documentation in Browser
if [ $open_docs_in_browser -eq 1 ]; then
	echo -e "${term_notice_setup}Opening documentation in browser..."
	launch_cmd="firefox --new-tab -url http://${host_ip}:${host_port}"
	gnome-terminal -- bash -c "${launch_cmd}; exec bash"
else
	echo -e "${term_warn_setup}Invalid Argument! Skipped opening documentation in browser!"
fi

# Finish
cd "${proj_dir}"
echo -e "${term_notice_setup}Setup complete."
