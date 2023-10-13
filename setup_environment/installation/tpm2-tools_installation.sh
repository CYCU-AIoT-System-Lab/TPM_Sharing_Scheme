# =====================================================================================================
# The following code is modified from the following source:
# 1. https://francislampayan.medium.com/how-to-setup-tpm-simulator-in-ubuntu-20-04-25ec673b88dc
# 2. https://gist.github.com/fbdlampayan/5ceaadda9c32d4c23478ee46f80207f9#file-gistfile1-txt
# =====================================================================================================
# Related Source:
# 1. https://github.com/tpm2-software/tpm2-tools/releases
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/12
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./tpm2-tools_installation.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

# set tpm2-tools version
ver=5.5

# acquire the current directory
path=$(pwd)

# download from official release
wget "https://github.com/tpm2-software/tpm2-tools/releases/download/$ver/tpm2-tools-$ver.tar.gz"

# extract, configure, install
tar -xzvf "tpm2-tools-$ver.tar.gz"
cd "tpm2-tools-$ver/"
./configure
make install

# Return to the original directory
cd $path

# test!
tpm2_pcrread
