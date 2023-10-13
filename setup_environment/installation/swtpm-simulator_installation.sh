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

echo -e "\n=====================================================================================================\nStart installing swtpm simulator ...\n=====================================================================================================\n"

# acquire the current directory
path=$(pwd)

# install dependencies for libtpms
apt install -y dpkg-dev debhelper libssl-dev libtool net-tools libfuse-dev libglib2.0-dev libgmp-dev expect libtasn1-dev socat python3-twisted gnutls-dev gnutls-bin  libjson-glib-dev gawk git python3-setuptools softhsm2 libseccomp-dev automake autoconf libtool gcc build-essential libssl-dev dh-exec pkg-config dh-autoreconf libtool-bin tpm2-tools libtss0 libtss2-dev dh-apparmor equivs devscripts swtpm-tools
# if can't get swtpm-tools, try to retart the entire process.?

# install libtpms
git clone https://github.com/stefanberger/libtpms.git
cd ./libtpms
bash ./autogen.sh --with-openssl
make dist
# automatically install dependencies for liptpms
mk-build-deps --install --root-cmd sudo --remove -y
dpkg-buildpackage -us -uc -j4
libtool --finish /usr/lib/x86_64-linux-gnu
apt install ../libtpms*.deb
cd $path

# install swtpm
git clone https://github.com/stefanberger/swtpm.git
cd ./swtpm
# automatically install dependencies for swtpm
mk-build-deps --install --root-cmd sudo --remove -y
dpkg-buildpackage -us -uc -j4
libtool --finish /usr/lib/x86_64-linux-gnu/
apt install ../swtpm*.deb

# modify bash profile
echo -e "\nexport TPM2TOOLS_TCTI=\"swtpm:port=2321\"\n" >> ~/.profile

# Return to the original directory
cd $path
