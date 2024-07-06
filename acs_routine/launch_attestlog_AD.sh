#!/bin/bash

script=$(realpath "$0")
script_path=$(dirname "$script")
dirname=$(basename "$script_path")
filename=$(basename "$0")
source "$script_path/../common/functions.sh"

echo_notice "$dirname" "$filename" "ACS DB parsing"

sudo systemctl restart mysql
while true; do
    output=$($script_path/attestlog_AD/attestlog_AD -H "$MYSQL_HOST" -u "$MYSQL_USER" -p "$MYSQL_PASSWORD" -d "$MYSQL_DATABASE" -P "$MYSQL_PORT")
    echo $output
    result_code=${output:0:1}
    if [ $result_code -eq "2" ]; then
        echo "Anomaly in remote attestation, send SIGUSR1 to alive traffic_AD"
        sudo kill -USR1 $(pidof traffic_AD)
    fi
    sleep $interval
done
