# =====================================================================================================
# The following code is modified from the following source:
# 1. https://francislampayan.medium.com/how-to-setup-tpm-simulator-in-ubuntu-20-04-25ec673b88dc
# 2. https://gist.github.com/fbdlampayan/5ceaadda9c32d4c23478ee46f80207f9#file-gistfile1-txt
# =====================================================================================================
# Related Source:
# 1. https://sourceforge.net/projects/ibmswtpm2/
# 2. https://sourceforge.net/projects/ibmtpm20tss/
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/11
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./TPM-simulator_installation.sh"
# =====================================================================================================
# Issue:
# tpm_server daemon can't be started
# =====================================================================================================
#!/bin/bash

echo -e "\n=====================================================================================================\nStart installing TPM-simulator_installation ...\n=====================================================================================================\n"

# set IBM simulator version
ver=1682

# acquire the current directory
path=$(pwd)

# install dependencies
apt-get install -y lcov \
pandoc autoconf-archive liburiparser-dev \
libdbus-1-dev libglib2.0-dev dbus-x11 \
libssl-dev autoconf automake \
libtool pkg-config gcc \
libcurl4-gnutls-dev libgcrypt20-dev libcmocka-dev uthash-dev \

# download TPM simulator
wget "https://jaist.dl.sourceforge.net/project/ibmswtpm2/ibmtpm$ver.tar.gz"

# create installation directory to extract towards into
mkdir "ibmtpm$ver"
cd "ibmtpm$ver"
tar -xzvf "../ibmtpm$ver.tar.gz"

# enter src/ directory and execute build
cd src/
make

# copy the built executable to your bin directory
cp tpm_server /usr/local/bin

# configure TPM simulator as a daemon service in ubuntu
echo -e "\nConfigure TPM simulator as a daemon service in ubuntu\n[Unit]\nDescription=TPM2.0 Simulator Server daemon\nBefore=tpm2-abrmd.service\n[Service]\nExecStart=/usr/local/bin/tpm_server\nRestart=always\nEnvironment=PATH=/usr/bin:/usr/local/bin\n[Install]\nWantedBy=multi-user.target\n" >> /lib/systemd/system/tpm-server.service

# reload daemon and start the service
systemctl daemon-reload
systemctl start tpm-server.service

# check its status, if all is fine should be in Active state
# service tpm-server status
tpm_server_status=$(systemctl show -p SubState --value tpm-server)
if [ $tpm_server_status == "running" ]; then
    echo "TPM server is installed and running ..."
else
    echo "TPM server is not running"
    exit 1
fi

# Return to the original directory
cd $path