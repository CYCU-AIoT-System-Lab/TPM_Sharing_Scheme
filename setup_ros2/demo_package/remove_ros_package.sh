#!/bin/bash
source common.sh

echo "> Removing package in $package_dir ..."
rm -rf $package_dir || { echo "Package doesn't exist at $package_dir ..."; }
