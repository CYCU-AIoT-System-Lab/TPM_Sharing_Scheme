#!/bin/bash

source "../common/functions.sh"
source "./function_boot.sh"
load_preset "./config.ini"

echo -e "\
[Unit]\n\
Description=TPM Sharing Scheme SWTPM Measured Boot Chain (MBC) perform at startup/boot required SWTPM server service\n\
After=multi-user.target\n\
StartLimitIntervalSec=0\n\
\n\
[Service]\n\
User=root\n\
Type=simple\n\
KillMode=mixed\n\
Restart=always\n\
#RestartSec=1\n\
WorkingDirectory=$PWD\n\
ExecStart=$LAUNCH_SCRIPT_1\n\
TimeoutStartSec=infinity\n\
\n\
[Install]\n\
WantedBy=multi-user.target\
" > $SERVICE_FILE_1
sudo mv $SERVICE_FILE_1 $SYSTEMD_DIR
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_FILE_1
sudo systemctl start $SERVICE_FILE_1

echo -e "\
[Unit]\n\
Description=SWTPM server required startup trigger service\n\
After=multi-user.target $SERVICE_FILE_1\n\
StartLimitIntervalSec=0\n\
\n\
[Service]\n\
User=$USER\n\
Type=simple\n\
KillMode=mixed\n\
ExecStartPre=/bin/sleep 1\n\
#Restart=always\n\
#RestartSec=1\n\
WorkingDirectory=$PWD\n\
ExecStart=$LAUNCH_SCRIPT_2\n\
TimeoutStartSec=infinity\n\
\n\
[Install]\n\
WantedBy=multi-user.target\
" > $SERVICE_FILE_2
sudo mv $SERVICE_FILE_2 $SYSTEMD_DIR
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_FILE_2
sudo systemctl start $SERVICE_FILE_2

echo "waiting for swtpm to be launched and online"
sleep 2
echo "activating swtpm server service"
bash $LAUNCH_SCRIPT_2
