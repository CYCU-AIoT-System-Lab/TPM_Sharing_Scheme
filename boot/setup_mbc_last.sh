#!/bin/bash
#set -x

source "../common/functions.sh"
source "./function_boot.sh"
load_preset "./config.ini"

# note: remember to add fail check

CLI_INPUT_STR="$@"
CLI_INPUT_ARR=($CLI_INPUT_STR)

# >0. Checking existance
#   - TPM
#   - scripts
#   - create a demo dir_list.txt
if [[ "${CLI_INPUT_ARR[*]}" =~ "-v" ]]; then
    VERBOSE=true
fi
echo "some data" > $HASH_DEMO_DATA
echo $HASH_DEMO_DATA > $HASH_TARGET

# >1. If choosen SWTPM, do preperations:
#   - Modify parameters
#       - daemon setting added in this script
#       - filepath
#   - Call 2 daemon setup script: setup_swtpm_daemon.sh
#       - swtpm.service
#       - activate_swtpm.service
if [ $USE_SWTPM -ne 1 ]; then
    MBC_AFTER=""
    MBC_DELAY=""
else
    if [ $VERBOSE == true ]; then echo "do SWTPM preparation..."; fi
    #1
    MBC_AFTER="activate_swtpm.service"
    MBC_DELAY="ExecStartPre=/bin/sleep 1"
    #2
    bash setup_swtpm_daemon.sh
fi

# >2. Configure TPM
if [ $VERBOSE == true ]; then echo "Configuring TPM..."; fi
if [ $USE_SWTPM -ne 1 ]; then
    # new SWTPM have a specific daemon starting it up
    tpm2_startup -c
    # new SWTPM can't be R/W for now with following cmd
    tpm2_clear -c p
    tpm2_changeauth -c owner $TPM2_AUTH_OWNER
    tpm2_changeauth -c endorsement $TPM2_AUTH_ENDORSEMENT
    tpm2_changeauth -c lockout $TPM2_AUTH_LOCKOUT
    tpm2_nvdefine $TPM2_NVM_INDEX -C o -s 900 -a $TPM2_AUTH_ATTRIBUTE -P $TPM2_AUTH_OWNER -p $TPM2_AUTH_NV
else
    # new SWTPM use SD card to store hash instead
    > $SWTPM_NVM
fi
# now we can write to NVM

# >3. Read PCR out from TPM and find the index of the first non-zero byte
# IDX:PCR_VALUE
# 1:1234
# 2:5678
# 3:9ABC <-                     PCR_X_1
# 4:0000 <- PCR_IDX_INIT_ZERO
# Formula: PCR_X-new = hash( PCR_X_1 || files )
# If PCR_X-old != PCR_X-new => something being modified
if [ $VERBOSE == true ]; then echo "Reading PCR from TPM..."; fi
PCR_X_1="$INITIAL_ZERO_PCR" # last non-zero PCR
for i in $(seq $PCR_IDX_MIN $PCR_IDX_MAX); do
    PCR_TEMP="$(tpm2_pcrread $HASH:$i)"
    PCR_TEMP="${PCR_TEMP#*x}"
    if [[ "$PCR_TEMP" == "$INITIAL_ZERO_PCR" ]]; then
        PCR_IDX_INIT_ZERO=$i
        break
    fi
    PCR_X_1=$PCR_TEMP
    #echo $PCR_TEMP
done

# >4. Call hashing script
if [ $VERBOSE == true ]; then echo "Hashing target file..."; fi
. "$HASH_SCRIPT" "$HASH_TARGET" "$PCR_X_1" 1> /dev/null
FINAL_HASH_VALUE="$(echo $FINAL_HASH_VALUE | tr -d ' ')"
#echo $FINAL_HASH_VALUE
if [ $VERBOSE == true ]; then
    echo -e "First zero PCR index: \t$PCR_IDX_INIT_ZERO"; 
    echo -e "Generated PCR_HASH[$PCR_IDX_INIT_ZERO]: \t$FINAL_HASH_VALUE"
fi

# >5. Extend PCR index x with new value
if [ $VERBOSE == true ]; then echo "Extending PCR..."; fi
tpm2_pcrextend "$PCR_IDX_INIT_ZERO:$HASH=$FINAL_HASH_VALUE"

# >6. Write hash value to NVM
if [ $VERBOSE == true ]; then echo "Writing hash to NVM..."; fi
HASH_TMP_FILE="hash_value.txt.tmp"
echo "$FINAL_HASH_VALUE" > $HASH_TMP_FILE
if [ $USE_SWTPM -ne 1 ]; then
    tpm2_nvwrite $TPM2_NVM_INDEX -P $TPM2_AUTH_NV -i $HASH_TMP_FILE
else
    cp $HASH_TMP_FILE $SWTPM_NVM
fi
rm $HASH_TMP_FILE

# >7. Set MBC script to run at startup/boot
if [ $VERBOSE == true ]; then echo "Setting MBC script to run at startup/boot..."; fi
echo -e "\
[Unit]\n\
Description=TPM Sharing Scheme SWTPM Measured Boot Chain (MBC) perform at startup/boot service\n\
After=multi-user.target $MBC_AFTER\n\
StartLimitIntervalSec=0\n\
\n\
[Service]\n\
User=$USER\n\
Type=simple\n\
KillMode=mixed\n\
#Restart=always\n\
#RestartSec=1\n\
WorkingDirectory=$PWD\n\
ExecStart=$LAUNCH_SCRIPT_3\n\
TimeoutStartSec=infinity\n\
$MBC_DELAY\n\
\n\
[Install]\n\
WantedBy=multi-user.target\
" > $SERVICE_FILE_3
sudo mv $SERVICE_FILE_3 $SYSTEMD_DIR
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_FILE_3
sudo systemctl start $SERVICE_FILE_3

echo "MBC script finished!"
