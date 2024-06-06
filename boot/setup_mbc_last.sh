#!/bin/bash
#set -x

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
PCR_IDX_INIT_ZERO=0
HASH="sha256" # can be sha1, sha256
PCR_IDX_MIN=0
PCR_IDX_MAX=23
INITIAL_ZERO_PCR="0000000000000000000000000000000000000000000000000000000000000000"

# >0. Checking existance
#    - TPM
#    - scripts

# >1. Configure TPM
echo "Configuring TPM..."
tpm2_startup -c
tpm2_clear -c p
tpm2_changeauth -c owner $TPM2_AUTH_OWNER
tpm2_changeauth -c endorsement $TPM2_AUTH_ENDORSEMENT
tpm2_changeauth -c lockout $TPM2_AUTH_LOCKOUT
tpm2_nvdefine $TPM2_NVM_INDEX -C o -s 900 -a $TPM2_AUTH_ATTRIBUTE -P $TPM2_AUTH_OWNER -p $TPM2_AUTH_NV
# now we can write to NVM

# >2. Read PCR out from TPM and find the index of the first non-zero byte
echo "Reading PCR from TPM..."
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

# >3. Call hashing script
echo "Hashing target file..."
. "$HASH_SCRIPT" "$HASH_TARGET" "$PCR_X_1" 1> /dev/null
FINAL_HASH_VALUE="$(echo $FINAL_HASH_VALUE | tr -d ' ')"
#echo $FINAL_HASH_VALUE
echo -e "First zero PCR index: \t$PCR_IDX_INIT_ZERO"
echo -e "Generated PCR_HASH[$PCR_IDX_INIT_ZERO]: \t$FINAL_HASH_VALUE"

# >4. Extend PCR index x with new value
echo "Extending PCR..."
tpm2_pcrextend "$PCR_IDX_INIT_ZERO:$HASH=$FINAL_HASH_VALUE"

# >5. Read just extended PCR
echo "Reading extended PCR..."
PCR_X="$(tpm2_pcrread $HASH:$PCR_IDX_INIT_ZERO)"
PCR_X="${PCR_X#*x}"

# >6. Write hash value to NVM
echo "Writing hash to NVM..."
HASH_TMP_FILE="hash_value.txt.tmp"
echo "$FINAL_HASH_VALUE" > $HASH_TMP_FILE
tpm2_nvwrite $TPM2_NVM_INDEX -P $TPM2_AUTH_NV -i $HASH_TMP_FILE
rm $HASH_TMP_FILE
echo "MBC script updated successfully!"
