# =====================================================================================================
# The following code is modified from the following source:
# 1. https://github.com/stefanberger/swtpm/wiki
# =====================================================================================================
# Related Source:
# 1. https://github.com/stefanberger/swtpm/releases
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/12
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./TPM-simulator_installation.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

# acquire the current directory
path=$(pwd)

apt-get install dh-autoreconf libssl-dev \
libtasn1-6-dev pkg-config libtpms-dev \
net-tools iproute2 libjson-glib-dev \
libgnutls28-dev expect gawk socat \
libseccomp-dev make -y
./autogen.sh --with-openssl --prefix=/usr
make -j4
make -j4 check
make install

# Return to the original directory
cd $path
