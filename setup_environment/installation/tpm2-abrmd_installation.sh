# =====================================================================================================
# The following code is modified from the following source:
# 1. https://francislampayan.medium.com/how-to-setup-tpm-simulator-in-ubuntu-20-04-25ec673b88dc
# 2. https://gist.github.com/fbdlampayan/5ceaadda9c32d4c23478ee46f80207f9#file-gistfile1-txt
# =====================================================================================================
# Related Source:
# 1. https://github.com/tpm2-software/tpm2-abrmd/releases
# 2. https://github.com/tpm2-software/tpm2-abrmd/blob/master/INSTALL.md
# 3. https://www.gnu.org/prep/standards/html_node/Directory-Variables.html
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/12
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./tpm2-abrmd_installation.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

echo -e "\n=====================================================================================================\nStart installing tpm2-abrmd ...\n=====================================================================================================\n"

# set tpm2-abrmd version
ver=3.0.0

# acquire the current directory
path=$(pwd)

# add tss user and group as that will be the one used by the package during bus name claim
# normally tpm2-tss installation will do this for you, but in case none yet then this is the command to do it
useradd --system --user-group tss

# download tpm2-abrmd
wget "https://github.com/tpm2-software/tpm2-abrmd/releases/download/$ver/tpm2-abrmd-$ver.tar.gz"

# extract, configure, and install
tar -xzvf "tpm2-abrmd-$ver.tar.gz"
cd "tpm2-abrmd-$ver"
ldconfig

# --with-dbuspolicydir this is th directory where a policy that will allow tss user account to claim a name on the D-Bus system bus.
# --with-systemdsystemunitdir: systemd unit directory that is needed for tpm2-abrmd daemon to be started as part of the boot process of your unit.
./configure --with-dbuspolicydir=/etc/dbus-1/system.d --with-systemdsystemunitdir=/usr/lib/systemd/system
make
make install

# add tpm2-abrmd into the system service
# during the build+install phase, a sample service definition is placed under /usr/local/share/dbus-1/system-services/
# copy it to the system-services directory
cp /usr/local/share/dbus-1/system-services/com.intel.tss2.Tabrmd.service /usr/share/dbus-1/system-services/

# restart DBUS
pkill -HUP dbus-daemon

# modify the tpm2-abrmd service configuration.
# there specify that the TCTI interface to be used is that of the software TPM
# this will make it act as the access broker for the software TPM
# comment out this line if you want to use the hardware TPM / the original settings
# for simulators
echo -e "[Unit]\nDescript=TPM2 Access Broker and Resource Management Daemon\n\n[Service]\nType=dbus\nRestart=always\nRestartSec=5\nBusName=com.intel.tss2.Tabrmd\nStandardOutput=syslog\nExecStart=/usr/local/sbin/tpm2-abrmd --tcti=\"libtss2-tcti-mssim.so.0:host=127.0.0.1,port=2321\"\nUser=tss\n\n[Install]\nWantedBy=multi-user.target\n" > /lib/systemd/system/tpm2-abrmd.service
# for hardware TPM
# echo -e "[Unit]\nDescript=TPM2 Access Broker and Resource Management Daemon\n# These settings are needed when using the device TCTI. If the\n# TCP mssim is used then the settings should be commented out.\nAfter=dev-tpm0.device\nRequires=dev-tpm0.device\n\n[Service]\nType=dbus\nBusName=com.intel.tss2.Tabrmd\nExecStart=/usr/local/sbin/tpm2-abrmd\nUser=tss\n\n[Install]\nWantedBy=multi-user.target\n" > /lib/systemd/system/tpm2-abrmd.service

# run the service and check its state. Should be in active state if all is good.
systemctl daemon-reload
systemctl start tpm2-abrmd.service
# service tpm2-abrmd status
tpm2_abrmd_status=$(systemctl show -p SubState --value tpm2-abrmd)
if [ $tpm2_abrmd_status == "running" ]; then
    echo "TPM2 Access Broker and Resource Management Daemon is installed and running ..."
else
    echo "TPM2 Access Broker and Resource Management Daemon is not running"
    exit 1
fi

# Return to the original directory
cd $path
