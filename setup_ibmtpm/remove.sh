#!/bin/bash

remove_path=(
    "/opt"
    "/var/www"
)

for i in "${remove_path[@]}"; do
    echo "Removing $i"
    sudo rm -rf $i
done

echo "Finished removing paths"
