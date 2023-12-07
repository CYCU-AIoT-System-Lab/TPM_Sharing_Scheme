#!/bin/bash
#!/bin/bash -x # for debugging

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
install_dependencies=$(awk -F "=" '/^install_dependencies/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
perform_clean=$(awk -F "=" '/^perform_clean/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
perform_build=$(awk -F "=" '/^perform_build/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")
build_for_debug=$(awk -F "=" '/^build_for_debug/ {gsub(/[ \t ]/, "", $2); print $2}' "${conf_file}")

# Option Conditioning
# -----------------------------
compile_client=$((perform_build * run_client))
compile_server=$((perform_build * run_server))
build_for_debug=$((perform_build * build_for_debug))

# Show Options
# -----------------------------
echo -e "${term_notice}Settings in ${conf_file}:"
echo -e "run_client:           ${run_client}"
echo -e "run_server:           ${run_server}"
echo -e "install_dependencies: ${install_dependencies}"
echo -e "perform_clean:        ${perform_clean}"
echo -e "perform_build:        ${perform_build}"
echo -e "build_for_debug:      ${build_for_debug}"
echo -e "compile_client:       ${compile_client}"
echo -e "compile_server:       ${compile_server}"

# Main Flow
# -----------------------------

#! Setup Directories
build_dir="${proj_dir}/build"
bin_dir="${proj_dir}/bin"
echo -e "${term_notice}Creating directories: ${build_dir}, ${bin_dir}"
mkdir $build_dir
mkdir $bin_dir

#! Install Dependencies
if [ $install_dependencies -eq 1 ]; then
	echo -e "${term_notice}Installing dependencies..."
	sudo apt-get update
	sudo apt-get update -y --fix-missing
	sudo apt-get install -y neovim cmake build-essential
elif [ $install_dependencies -eq 0 ]; then
	echo -e "${term_notice}Skipped installing dependencies!"
else
	echo -e "${term_warn}Invalid Argument! Skipped installing dependencies!"
fi

#! Clean
cd "${proj_dir}/build"
if [ $perform_clean -eq 1 ]; then
	echo -e "${term_notice}Cleaning project..."
	rm -rf "${proj_dir}/build/*"
elif [ $perform_clean -eq 0 ]; then
	echo -e "${term_notice}Skipped cleaning project!"
else
	echo -e "${term_warn}Invalid Argument! Skipped cleaning project!"
fi

#! Build
cd "${proj_dir}/build"

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

# Build
if [ $build_for_debug -eq 1 ]; then
	echo -e "${term_notice}Generating makefile for debug..."
	cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=ON
	echo -e "${term_notice}Building project..."
	make -j"$(nproc)"
	echo -e "${term_notice}Installing project..."
	make install
elif [ $build_for_debug -eq 0 ]; then
	echo -e "${term_notice}Generating makefile for release..."
	cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=.. -DBUILD_SHARED_LIBS=ON
	echo -e "${term_notice}Building project..."
	make -j"$(nproc)"
	echo -e "${term_notice}Installing project..."
	make install
else
	echo -e "${term_warn}Invalid Argument! Skipped generating makefile!"
fi

#! Run

# Run Client
cd "${proj_dir}/bin"
if [ $run_client -eq 1 ]; then
	echo -e "${term_notice}Running client on new terminal..."
	#./client
	launch_cmd="echo \"${term_notice}Starting client...\"; ./client"
	gnome-terminal -t "server_com client" --active -- bash -c "${launch_cmd}; exec bash"
elif [ $run_client -eq 0 ]; then
	echo -e "${term_notice}Skipped running client!"
else
	echo -e "${term_warn}Invalid Argument! Skipped running client!"
fi

# Run Server
cd "${proj_dir}/bin"
if [ $run_server -eq 1 ]; then
	echo -e "${term_notice}Running server on new terminal..."
	#./server
	launch_cmd="echo \"${term_notice}Starting server...\"; ./server"
	gnome-terminal -t "server_com server" --active -- bash -c "${launch_cmd}; exec bash"
elif [ $run_server -eq 0 ]; then
	echo -e "${term_notice}Skipped running server!"
else
	echo -e "${term_warn}Invalid Argument! Skipped running server!"
fi

# Finish
cd "${proj_dir}"
echo -e "${term_notice}Setup complete."
