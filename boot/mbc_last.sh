#!/bin/bash
#set -x

TPM2_AUTH_NV="nv123"
TPM2_NVM_INDEX="0x1500016"
HASH_SCRIPT="./hash.sh"
HASH_TARGET="dir_list.txt"
HASH="sha256" # can be sha1, sha256
PCR_IDX_INIT_ZERO=0
PCR_IDX_MIN=0
PCR_IDX_MAX=23
INITIAL_ZERO_PCR="0000000000000000000000000000000000000000000000000000000000000000"
INTEGRITY_CHECK_PASS=false

# >0. Checking existance
#    - TPM
#    - scripts

# >1. Find the index of last non-zero PCR
echo "Finding the index of last non-zero PCR..."
for i in $(seq $PCR_IDX_MIN $PCR_IDX_MAX); do
    PCR_TEMP="$(tpm2_pcrread $HASH:$i)"
    PCR_TEMP="${PCR_TEMP#*x}"
    if [[ "$PCR_TEMP" == "$INITIAL_ZERO_PCR" ]]; then
        PCR_IDX_INIT_ZERO=$((i-1))
        break
    fi
done
echo -e "Last non-zero PCR index: \t$PCR_IDX_INIT_ZERO"

# >2. Read PCR out from TPM
echo "Reading PCR from TPM..."
PCR_IDX_INIT_ZERO_1=$((PCR_IDX_INIT_ZERO-1))
if [[ $PCR_IDX_INIT_ZERO_1 -lt $PCR_IDX_MIN ]]; then
    PCR_X_1="$INITIAL_ZERO_PCR"
else
    PCR_X_1="$(tpm2_pcrread $HASH:$PCR_IDX_INIT_ZERO_1)"
    PCR_X_1="${PCR_X_1#*x}"
fi
PCR_X="$(tpm2_pcrread $HASH:$PCR_IDX_INIT_ZERO)"
PCR_X="${PCR_X#*x}"
echo -e "PCR_HASH[$PCR_IDX_INIT_ZERO_1]: \t$PCR_X_1"
echo -e "PCR_HASH[$PCR_IDX_INIT_ZERO]: \t$PCR_X"

# >3. Call hashing script
echo "Hashing target file..."
. "$HASH_SCRIPT" "$HASH_TARGET" "$PCR_X_1" 1> /dev/null
FINAL_HASH_VALUE="$(echo $FINAL_HASH_VALUE | tr -d ' ')"
echo -e "Generated PCR_HASH[$PCR_IDX_INIT_ZERO]: \t$FINAL_HASH_VALUE"

# >4. Read hash out from NVM
echo "Reading hash from NVM..."
NVM_HASH_TMP_FILE="nvm_hash_value.txt.tmp"
tpm2_nvread "$TPM2_NVM_INDEX" -s 64 -o 0 -P "$TPM2_AUTH_NV" -o "$NVM_HASH_TMP_FILE"
NVM_HASH="$(cat $NVM_HASH_TMP_FILE)"
rm "$NVM_HASH_TMP_FILE"
echo -e "Hash value in NVM: \t$NVM_HASH"
if [[ "$NVM_HASH" == "$FINAL_HASH_VALUE" ]]; then
    echo "Hash value in NVM is the same as the generated one"
    INTEGRITY_CHECK_PASS=true
else
    echo "Hash value in NVM is different from the generated one"
fi

# >5. Extend PCR index if needed
if [[ "$INTEGRITY_CHECK_PASS" == true ]]; then
    echo "No need to extend PCR, hash value is the same as the one previously calculated and stored in NVM"
    echo "MBC: File integrity check completed successfully!"
else
    echo "Extending PCR..."
    #tpm2_pcrextend "$PCR_IDX_INIT_ZERO:$HASH=$FINAL_HASH_VALUE"
    echo "MBC: File integrity check failed!"
fi
