#!/bin/bash

# note: remember to add fail check

PCR_IDX_INIT_ZERO=0
HASH="sha256" # can be sha1, sha256


HASH_SCRIPT="./hash.sh"
HASH_TARGET="dir_list.txt"
MBC_SCRIPT="./mbc_last.sh"
PCR_IDX_MIN=0
PCR_IDX_MAX=23
INITIAL_ZERO_PCR="0000000000000000000000000000000000000000000000000000000000000000"

# >0. Checking existance
#    - TPM
#    - scripts

# >1. Read PCR out from TPM and find the index of the first non-zero byte
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

# >2. Call hashing script
echo "Hashing target file..."
. "$HASH_SCRIPT" "$HASH_TARGET" "$PCR_X_1" 1> /dev/null
FINAL_HASH_VALUE="$(echo $FINAL_HASH_VALUE | tr -d ' ')"
echo $FINAL_HASH_VALUE
echo -e "First zero PCR index: \t$PCR_IDX_INIT_ZERO"
echo -e "Generated PCR[$PCR_IDX_INIT_ZERO]: \t$FINAL_HASH_VALUE"

# >3. Extend PCR index x with new value
echo "Extending PCR..."
tpm2_pcrextend "$PCR_IDX_INIT_ZERO:$HASH=$FINAL_HASH_VALUE"

# >4. Read just extended PCR
echo "Reading extended PCR..."
PCR_X="$(tpm2_pcrread $HASH:$PCR_IDX_INIT_ZERO)"
PCR_X="${PCR_X#*x}"

# >5. Write PCR and index to MBC script
echo "Writing PCR and index to MBC script..."
sed -i "3s/PCR_IDX_INIT_ZERO=.*/PCR_IDX_INIT_ZERO=$PCR_IDX_INIT_ZERO/" "$MBC_SCRIPT"
sed -i "4s/PCR_X_1=.*/PCR_X_1=\"$PCR_X_1\"/" "$MBC_SCRIPT"
sed -i "5s/PCR_X=.*/PCR_X=\"$PCR_X\"/" "$MBC_SCRIPT"


echo "MBC script updated successfully!"
