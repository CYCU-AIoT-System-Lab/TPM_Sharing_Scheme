#!/bin/bash

download_dir="/home/user/Downloads"

echo -e "\n====================================================\nSetup IBMTPM\n====================================================\n"

cp ./*.sh "${download_dir}"

chmod +x "${download_dir}/install_req.sh"
chmod +x "${download_dir}/install_ibmswtpm.sh"
chmod +x "${download_dir}/install_ibmtpm2tss.sh"
chmod +x "${download_dir}/install_ibmacs.sh"

bash "${download_dir}/install_req.sh"
bash "${download_dir}/install_ibmswtpm.sh"
bash "${download_dir}/install_ibmtpm2tss.sh"
bash "${download_dir}/install_ibmacs.sh"

echo -e "\n====================================================\nSetup IBMTPM Complete\n====================================================\n"
