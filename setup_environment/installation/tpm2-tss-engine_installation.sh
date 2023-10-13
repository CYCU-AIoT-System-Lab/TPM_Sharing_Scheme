# =====================================================================================================
# The following code is modified from the following source:
# 1. https://francislampayan.medium.com/how-to-setup-tpm-simulator-in-ubuntu-20-04-25ec673b88dc
# 2. https://gist.github.com/fbdlampayan/5ceaadda9c32d4c23478ee46f80207f9#file-gistfile1-txt
# =====================================================================================================
# Related Source:
# 1. https://github.com/tpm2-software/tpm2-tss-engine/releases
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/12
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./tpm2-tss-engine_installation.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

echo -e "\n=====================================================================================================\nStart installing tpm2-tss-engine ...\n=====================================================================================================\n"

# set tpm2-tss-engine version
ver=1.2.0

# acquire the current directory
path=$(pwd)

# get version compatible, for this article's example its 1.2.0
wget "https://github.com/tpm2-software/tpm2-tss-engine/releases/download/v$ver/tpm2-tss-engine-$ver.tar.gz"

# extract, configure, install
tar -xzvf "tpm2-tss-engine-$ver.tar.gz"
cd "tpm2-tss-engine-$ver/"
./configure
make install
ldconfig

# Return to the original directory
cd $path

# Test if the engine is installed properly with openssl
openssl rand -engine tpm2tss -hex 20
