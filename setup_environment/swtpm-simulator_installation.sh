# =====================================================================================================
# The following code is modified from the following source:
# 1. https://github.com/stefanberger/swtpm/wiki
# 2. https://stackoverflow.com/questions/71220170/how-to-install-start-using-swtpm-on-linux
# =====================================================================================================
# Related Source:
# 1. https://github.com/stefanberger/libtpms/releases
# 2. https://github.com/stefanberger/swtpm/releases
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/12
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./swtpm-simulator_installation.sh"
# =====================================================================================================
# Issue:
# 1. Can't use activation command to start swtpm socket with the following error:
#     "swtpm: Could not open TCP socket: Address already in use"
# =====================================================================================================
#!/bin/bash

# acquire the current directory
path=$(pwd)

# install dependencies for libtpms
apt -y install dpkg-dev debhelper libssl-dev libtool net-tools libfuse-dev libglib2.0-dev libgmp-dev expect libtasn1-dev socat python3-twisted gnutls-dev gnutls-bin  libjson-glib-dev gawk git python3-setuptools softhsm2 libseccomp-dev automake autoconf libtool gcc build-essential libssl-dev dh-exec pkg-config dh-autoreconf libtool-bin tpm2-tools libtss0 libtss2-dev

# install libtpms
git clone https://github.com/stefanberger/libtpms.git
cd ./libtpms
cd ./autogen.sh --with-openssl
make dist
dpkg-buildpackage -us -uc -j4
libtool --finish /usr/lib/x86_64-linux-gnu
apt install ../libtpms*.deb
cd $path

# install swtpm dependencies
apt-get install -y dh-apparmor swtpm-tools

# install swtpm
git clone https://github.com/stefanberger/swtpm.git
cd ./swtpm
dpkg-buildpackage -us -uc -j4
libtool --finish /usr/lib/x86_64-linux-gnu/
apt install ../swtpm*.deb

# modify bash profile
echo -e "\nexport TPM2TOOLS_TCTI=\"swtpm:port=2321\"\n" >> ~/.profile

# # download TPM simulator
# ./autogen.sh --with-openssl --prefix=/usr
# make -j4
# make -j4 check
# make install

# Return to the original directory
cd $path

# Activation Command\
# netstat -tuln | grep -E "2321|2322" # check if the port is occupied
# mkdir /tmp/tpmrm0
# swtpm socket --tpmstate dir="/tmp/tpm0" --tpm2 --flags not-need-init,startup-clear --server type=tcp,port=2321 --ctrl type=tcp,port=2322
