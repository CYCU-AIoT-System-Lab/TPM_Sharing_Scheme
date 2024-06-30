#!/bin/bash

source "../common/functions.sh"
source "./function_boot.sh"
load_preset "./config.ini"

tpm2_startup -c
