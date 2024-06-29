#!/bin/bash
#set -x

# Use new SWTPM installed in `update_swtpm` submodule instead of HWTPM
USE_SWTPM=1
SWTPM_SERVER_PORT=2321
SWTPM_CTRL_PORT=2322

# note: remember to add fail check

TPM2_AUTH_OWNER="owner123"
TPM2_AUTH_ENDORSEMENT="endorsement123"
TPM2_AUTH_LOCKOUT="lockout123"
TPM2_AUTH_ATTRIBUTE="authwrite|authread|read_stclear"
TPM2_AUTH_NV="nv123"
TPM2_NVM_INDEX="0x1500016"
HASH_SCRIPT="./hash.sh"
HASH_TARGET="dir_list.txt"
MBC_SCRIPT="./mbc_last.sh"
SERVICE_FILE="mbc_last.service"
SYSTEMD_DIR="/etc/systemd/system"
PCR_IDX_INIT_ZERO=0
HASH="sha256" # can be sha1, sha256
PCR_IDX_MIN=0
PCR_IDX_MAX=23
INITIAL_ZERO_PCR="0000000000000000000000000000000000000000000000000000000000000000"
VERBOSE=false
CLI_INPUT_STR="$@"
CLI_INPUT_ARR=($CLI_INPUT_STR)

# >0. Checking existance
#    - TPM
#    - scripts
if [[ "${CLI_INPUT_ARR[*]}" =~ "-v" ]]; then
    VERBOSE=true
fi

# >1. If choosen SWTPM, do preperations:
#   - modify script parameters
#       - launch_swtpm.sh
#       - activate_swtpm.sh
#   - Call 2 daemon setup script: setup_swtpm_daemon.sh
#       - swtpm.service
#       - activate_swtpm.service
#   - Modify daemon added in this script
MBC_AFTER=""
MBC_DELAY=""
if [ $USE_SWTPM -eq 1 ]; then
    if [ $VERBOSE == true ]; then echo "do SWTPM preparation..."; fi
    #1
    sed "/SERVER_PORT=/c SERVER_PORT=$SWTPM_SERVER_PORT" launch_swtpm.sh
    sed "/CTRL_PORT=/c CTRL_PORT=$SWTPM_CTRL_PORT" launch_swtpm.sh
    sed "/SERVER_PORT=/c SERVER_PORT=$SWTPM_SERVER_PORT" activate_swtpm.sh
    #2
    bash setup_swtpm_daemon.sh
    #3
    MBC_AFTER="activate_swtpm.service"
    MBC_DELAY="ExecStartPre=/bin/sleep 1"
fi

# >2. Configure TPM
if [ $VERBOSE == true ]; then echo "Configuring TPM..."; fi
tpm2_startup -c
tpm2_clear -c p
tpm2_changeauth -c owner $TPM2_AUTH_OWNER
tpm2_changeauth -c endorsement $TPM2_AUTH_ENDORSEMENT
tpm2_changeauth -c lockout $TPM2_AUTH_LOCKOUT
tpm2_nvdefine $TPM2_NVM_INDEX -C o -s 900 -a $TPM2_AUTH_ATTRIBUTE -P $TPM2_AUTH_OWNER -p $TPM2_AUTH_NV
# now we can write to NVM

# >3. Read PCR out from TPM and find the index of the first non-zero byte
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
tpm2_nvwrite $TPM2_NVM_INDEX -P $TPM2_AUTH_NV -i $HASH_TMP_FILE
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
ExecStart=$MBC_SCRIPT\n\
TimeoutStartSec=infinity\n\
$MBC_DELAY\n\
\n\
[Install]\n\
WantedBy=multi-user.target\
" > $SERVICE_FILE
sudo mv $SERVICE_FILE $SYSTEMD_DIR
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_FILE
sudo systemctl start $SERVICE_FILE

echo "MBC script updated successfully!"
