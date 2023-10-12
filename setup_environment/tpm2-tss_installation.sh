# =====================================================================================================
# The following code is modified from the following source:
# 1. https://francislampayan.medium.com/how-to-setup-tpm-simulator-in-ubuntu-20-04-25ec673b88dc
# 2. https://gist.github.com/fbdlampayan/5ceaadda9c32d4c23478ee46f80207f9#file-gistfile1-txt
# =====================================================================================================
# Related Source:
# 1. https://github.com/tpm2-software/tpm2-tss/releases
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/12
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./tpm2-tss_installation.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

# acquire the current directory
path=$(pwd)

# install json-c on top of the other dependencies we've installed from the previous step
apt-get install -y libjson-c-dev

# download release 3.1.0 of tpm2-tss
wget https://github.com/tpm2-software/tpm2-tss/releases/download/3.2.2/tpm2-tss-3.2.2.tar.gz

# extract, configure and build
tar -xzvf tpm2-tss-3.2.2.tar.gz
cd tpm2-tss-3.2.2/
./configure
make install

# Update the run-time bindings before executing a program that links against the libraries, as prep for the next item that we will install
ldconfig

# Return to the original directory
cd $path