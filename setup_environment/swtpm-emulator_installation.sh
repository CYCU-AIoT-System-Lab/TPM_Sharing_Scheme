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

# Install dependencies
apt-get install -y libtasn1-devel expect socat python3-twisted fuse-devel glib2-devel gnutls-devel gnutls-utils gnutls json-glib-devel
./autogen.sh --with-openssl --prefix=/usr
make -j4
# Depending on how many CPUs you have, choose the -j parameter 
# carefully for the next command. The tests work on Raspberry PI 2
# for example but only if run with '-j1', otherwise timeouts may
# occur.
make -j4 check
make install

# Return to the original directory
cd $path
