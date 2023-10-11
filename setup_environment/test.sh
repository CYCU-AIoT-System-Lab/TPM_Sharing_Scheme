if systemctl is-active --quiet systemd-networkd; then
    echo "Docker is running"
else
    echo "Docker is not running"
fi