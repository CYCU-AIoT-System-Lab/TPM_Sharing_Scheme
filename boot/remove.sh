#!/bin/bash

SERVICE_FILE="mbc_last.service"
SYSTEMD_DIR="/etc/systemd/system"

echo "Removing MBC from boot sequence..."
sudo systemctl stop $SERVICE_FILE
sudo systemctl disable $SERVICE_FILE
sudo rm "$SYSTEMD_DIR/$SERVICE_FILE"
sudo systemctl daemon-reload

echo "MBC removed from boot sequence successfully!"
