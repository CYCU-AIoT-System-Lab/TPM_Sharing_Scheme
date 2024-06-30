#!/bin/bash
#set -x

source "../common/functions.sh"
source "./function_boot.sh"
load_preset "./config.ini"

INTEGRITY_CHECK_PASS=false
CLI_INPUT_STR="$@"
CLI_INPUT_ARR=($CLI_INPUT_STR)

# >0. Checking existance
#    - TPM
#    - scripts
if [[ "${CLI_INPUT_ARR[*]}" =~ "-v" ]]; then
    VERBOSE=true
fi

# >1. Find the index of last non-zero PCR
# IDX:PCR_VALUE
# 1:1234
# 2:5678 <- $PCR_IDX_INIT_ZERO_2 == PCR_X_1
# 3:9ABC <- $PCR_IDX_INIT_ZERO_1 == PCR_X
# 4:0000 <- $PCR_IDX_INIT_ZERO
# Formula: PCR_X-new = hash( PCR_X_1 || files )
# If PCR_X-old != PCR_X-new => something being modified
if [ $VERBOSE == true ]; then echo "Finding the index of last non-zero PCR..."; fi
for i in $(seq $PCR_IDX_MIN $PCR_IDX_MAX); do
    PCR_TEMP="$(tpm2_pcrread $HASH:$i)"
    PCR_TEMP="${PCR_TEMP#*x}"
    if [[ "$PCR_TEMP" == "$INITIAL_ZERO_PCR" ]]; then
        PCR_IDX_INIT_ZERO=$i
        break
    fi
done
PCR_IDX_INIT_ZERO_1=$((PCR_IDX_INIT_ZERO-1))
PCR_IDX_INIT_ZERO_2=$((PCR_IDX_INIT_ZERO-2))
if [ $VERBOSE == true ]; then
    echo "Last non-zero PCR index: $PCR_IDX_INIT_ZERO_1"
fi

# >2. Read PCR out from TPM
if [ $VERBOSE == true ]; then echo "Reading PCR from TPM..."; fi
if [[ $PCR_IDX_INIT_ZERO -le $PCR_IDX_MIN ]]; then
    echo "Please perform setup with \`setup_mbc_last.sh\` first"
    exit 1
    #PCR_X_1="$INITIAL_ZERO_PCR"
    #PCR_X="$INITIAL_ZERO_PCR"
elif [[ $PCR_IDX_INIT_ZERO_1 -le $PCR_IDX_MIN ]]; then
    PCR_X_1="$INITIAL_ZERO_PCR"
    PCR_X="$(tpm2_pcrread $HASH:$PCR_IDX_INIT_ZERO_1)"
    PCR_X="${PCR_X#*x}"
else
    PCR_X_1="$(tpm2_pcrread $HASH:$PCR_IDX_INIT_ZERO_2)"
    PCR_X_1="${PCR_X_1#*x}"
    PCR_X="$(tpm2_pcrread $HASH:$PCR_IDX_INIT_ZERO_1)"
    PCR_X="${PCR_X#*x}"
fi
if [ $VERBOSE == true ]; then
    echo -e "PCR_HASH[$PCR_IDX_INIT_ZERO_2]: \t$PCR_X_1"
    echo -e "PCR_HASH[$PCR_IDX_INIT_ZERO_1]: \t$PCR_X"
fi

# >3. Call hashing script
if [ $VERBOSE == true ]; then echo "Hashing target file..."; fi
. "$HASH_SCRIPT" "$HASH_TARGET" "$PCR_X_1" 1> /dev/null
FINAL_HASH_VALUE="$(echo $FINAL_HASH_VALUE | tr -d ' ')"
if [ $VERBOSE == true ]; then echo -e "Generated PCR_HASH[$PCR_IDX_INIT_ZERO_1]: \t$FINAL_HASH_VALUE"; fi

# >4. Read hash out from NVM
if [ $VERBOSE == true ]; then echo "Reading hash from NVM..."; fi
NVM_HASH_TMP_FILE="nvm_hash_value.txt.tmp"
if [ $USE_SWTPM -ne 1 ]; then
    tpm2_nvread "$TPM2_NVM_INDEX" -s 64 -o 0 -P "$TPM2_AUTH_NV" -o "$NVM_HASH_TMP_FILE"
else
    cp $SWTPM_NVM $NVM_HASH_TMP_FILE
fi
NVM_HASH="$(cat $NVM_HASH_TMP_FILE)"
rm "$NVM_HASH_TMP_FILE"
if [ $VERBOSE == true ]; then echo -e "Hash value in NVM: \t$NVM_HASH"; fi
if [[ "$NVM_HASH" == "$FINAL_HASH_VALUE" ]]; then
    if [ $VERBOSE == true ]; then
        echo "Hash value in NVM is the same as the generated one"
    fi
    INTEGRITY_CHECK_PASS=true
else
    if [ $VERBOSE == true ]; then
        echo "Hash value in NVM is different from the generated one"
    fi
fi

# >5. Extend PCR index if needed
if [[ "$INTEGRITY_CHECK_PASS" == true ]]; then
    echo "No need to extend PCR, hash value is the same as the one previously calculated and stored in NVM"
    echo "MBC: File integrity check completed successfully!"
else
    echo "Hash value is different from the one previously calculated and stored in NVM"
    echo "Extending PCR..."
    tpm2_pcrextend "$PCR_IDX_INIT_ZERO_1:$HASH=$FINAL_HASH_VALUE"
    echo "Updating hash value in NVM..."
    HASH_TMP_FILE="hash_value.txt.tmp"
    echo "$FINAL_HASH_VALUE" > $HASH_TMP_FILE
    if [ $USE_SWTPM -ne 1 ]; then
        tpm2_nvwrite $TPM2_NVM_INDEX -P $TPM2_AUTH_NV -i $HASH_TMP_FILE
    else
        cp $HASH_TMP_FILE $SWTPM_NVM
    fi
    rm $HASH_TMP_FILE
    echo "MBC: File integrity check failed!"
    exit 1
fi
