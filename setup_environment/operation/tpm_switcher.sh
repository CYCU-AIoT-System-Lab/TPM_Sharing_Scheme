# =====================================================================================================
# The following code is modified from the following source:
# =====================================================================================================
# Related Source:
# =====================================================================================================
# Modified by: Dachuan Chen
# Date: 2023/10/14
# Removed all "sudo" commands.
# Nevigate to your desired directory and run this script with "sudo ./tpm_daemen_killer.sh"
# =====================================================================================================
# Issue:
# =====================================================================================================
#!/bin/bash

# Port number
port=2321

echo -e "\n=====================================================================================================\nStart installing tpm_switcher ...\n=====================================================================================================\n"

# Manipulate services
# 1. tpm2-abrmd.service
# 2. tpm-server.service

# Stop services
systemctl stop tpm2-abrmd.service
systemctl stop tpm-server.service

# Message
echo -e "\n=====================================================================================================\nAll TPM daemons are stopped.\n=====================================================================================================\n"

# Adjust Settings
abrmdsettings=$(head -1 /lib/systemd/system/tpm2-abrmd.service)
if [ $abrmd == "Settings for SIMULATORS" ]; then
    echo -e "#Settings for PHYSICALDEVICE\n\n[Unit]\nDescript=TPM2 Access Broker and Resource Management Daemon\n# These settings are needed when using the device TCTI. If the\n# TCP mssim is used then the settings should be commented out.\nAfter=dev-tpm0.device\nRequires=dev-tpm0.device\n\n[Service]\nType=dbus\nBusName=com.intel.tss2.Tabrmd\nExecStart=/usr/local/sbin/tpm2-abrmd\nUser=tss\n\n[Install]\nWantedBy=multi-user.target\n" > /lib/systemd/system/tpm2-abrmd.service
elif [ $abrmd == "Settings for PHYSICALDEVICE" ]; then
    echo -e "#Settings for SIMULATORS\n\n[Unit]\nDescript=TPM2 Access Broker and Resource Management Daemon\n\n[Service]\nType=dbus\nRestart=always\nRestartSec=5\nBusName=com.intel.tss2.Tabrmd\nStandardOutput=syslog\nExecStart=/usr/local/sbin/tpm2-abrmd --tcti=\"libtss2-tcti-mssim.so.0:host=127.0.0.1,port=2321\"\nUser=tss\n\n[Install]\nWantedBy=multi-user.target\n" > /lib/systemd/system/tpm2-abrmd.service
else
    echo -e "Error: Unknown tpm2-abrmd settings."
fi

# Message
echo -e "\n=====================================================================================================\nApplied new settings.\n=====================================================================================================\n"

# Start services
systemctl daemon-reload
systemctl start tpm-server.service
kill -9 $(lsof -t -i:$port)
systemctl daemon-reload
systemctl start tpm2-abrmd.service

# Message
echo -e "\n=====================================================================================================\nAll TPM daemons are started.\n=====================================================================================================\n"
