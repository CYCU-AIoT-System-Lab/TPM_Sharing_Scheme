#!/bin/bash

# Setup Tasks (0=No, 1=Yes)
install_client=1
install_server=1

# Set working directory
proj_dir="${PWD}/server_com"

# Delete cmake subproject
awk '/add_subdirectory/d' "${PWD}/CMakeLists.txt"
