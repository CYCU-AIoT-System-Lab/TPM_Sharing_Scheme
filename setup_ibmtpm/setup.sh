#!/bin/bash

download_dir="/home/user/Downloads"

echo -e "\n====================================================\nSetup IBMTPM\n====================================================\n"

echo -e "Copying files to ${download_dir}..."
cp ./*.sh "${download_dir}"

echo -e "Changing permissions..."
chmod +x "${download_dir}/install_req.sh"
chmod +x "${download_dir}/install_ibmswtpm.sh"
chmod +x "${download_dir}/install_ibmtpm2tss.sh"
chmod +x "${download_dir}/install_ibmacs.sh"

echo -e "Installing IBMTPM..."
bash +x "${download_dir}/install_req.sh"
bash +x "${download_dir}/install_ibmswtpm.sh"
bash +x "${download_dir}/install_ibmtpm2tss.sh"
bash +x "${download_dir}/install_ibmacs.sh"

echo -e "\n====================================================\nSetup IBMTPM Complete\n====================================================\n"
