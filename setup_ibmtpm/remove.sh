#!/bin/bash

remove_path=(
    "/opt"
    "/var/www"
)

for i in "${remove_path[@]}"; do
    echo "Removing content in $i"
    cd $i
    sudo rm -rf *
done

echo "Finished removing paths"
