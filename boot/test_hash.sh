#!/bin/bash
# This script is used to test the consistency of hashing script

# ==================================================
# Test options -------------------------------------

test_total_count=100

test_default=true
test_ramdisk=true
test_trim=true
test_full=true
test_tpm_default=true
test_tpm_ramdisk=true
test_tpm_trim=true
test_tpm_full=true

# ==================================================
# Test functions -----------------------------------

total_test_count=8
success_count=0
test_no=0
test_result_hash=()

echo -e "Start testing hash.sh consecutively $test_total_count times for different options\n"
ideal_len=64
result_arr=()

execute_hash_with_args() {
    local script="$1"
    shift
    . "$script" "$@" > /dev/null
}

test_command_and_list_unique_cnt(){
    echo ""
    local result_arr=()
    local unique_result_arr=()
    for i in $(seq 1 $test_total_count); do
        echo -e "\e[1A\e[KTest $i/$test_total_count"
        execute_hash_with_args "$@"
        result_arr+=("$FINAL_HASH_VALUE")
    done
    unique_result_arr=($(echo "${result_arr[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))
    unique_count=${#unique_result_arr[@]}
    if [ $unique_count -eq 1 ]; then
        echo -e "\e[1A\e[K\033[32m\033[1mUnique count test passed\033[0m"
        success_count=$((success_count+1))
        test_result_hash+=("${unique_result_arr[0]}")
    else
        echo -e "\e[1A\e[K\033[31m\033[1mUnique count test failed\033[0m | Unique count: $unique_count"
    fi
    for index in "${!unique_result_arr[@]}"; do
        temp_item=${unique_result_arr[$index]}
        temp_len=${#unique_result_arr[$index]}
        temp_diff=$((ideal_len - temp_len))
        temp_cnt=$(grep -o "$temp_item" <<< "${result_arr[@]}" | wc -l)
        echo -e "Cnt $temp_cnt \tlen $temp_len \tlen_diff $temp_diff \t$temp_item"
    done
}

# test defualt
test_no=$((test_no+1))
if [ $test_default == true ]; then
    echo ">> $test_no Testing default hash.sh"
    time test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v"
else
    echo -e ">> $test_no Skip testing default hash.sh\n"
fi

# test with ramdisk
test_no=$((test_no+1))
if [ $test_ramdisk == true ]; then
    echo ">> $test_no Testing hash.sh with ramdisk"
    time test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-r"
else
    echo -e ">> $test_no Skip testing hash.sh with ramdisk\n"
fi

# test with trim
test_no=$((test_no+1))
if [ $test_trim == true ]; then
    echo ">> $test_no Testing hash.sh with trim"
    time test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-tr"
else
    echo -e ">> $test_no Skip testing hash.sh with trim\n"
fi

# test with ramdisk, trim
test_no=$((test_no+1))
if [ $test_full == true ]; then
    echo ">> $test_no Testing hash.sh with, ramdisk, trim"
    time test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-r" "-tr"
else
    echo -e ">> $test_no Skip testing hash.sh with pipe, ramdisk, trim\n"
fi

# test tpm default
test_no=$((test_no+1))
if [ $test_tpm_default == true ]; then
    echo ">> $test_no Testing hash.sh with tpm default"
    time test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-tpm"
else
    echo -e ">> $test_no Skip testing hash.sh with tpm default\n"
fi

# test tpm with ramdisk
test_no=$((test_no+1))
if [ $test_tpm_ramdisk == true ]; then
    echo ">> $test_no Testing hash.sh with tpm ramdisk"
    time test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-r" "-tpm"
else
    echo -e ">> $test_no Skip testing hash.sh with tpm ramdisk\n"
fi

# test tpm with trim
test_no=$((test_no+1))
if [ $test_tpm_trim == true ]; then
    echo ">> $test_no Testing hash.sh with tpm trim"
    time test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-tr" "-tpm"
else
    echo -e ">> $test_no Skip testing hash.sh with tpm trim\n"
fi

# test tpm with ramdisk, trim
test_no=$((test_no+1))
if [ $test_tpm_full == true ]; then
    echo ">> $test_no Testing hash.sh with tpm, ramdisk, trim"
    time test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-r" "-tr" "-tpm"
else
    echo -e ">> $test_no Skip testing hash.sh with tpm pipe, ramdisk, trim\n"
fi

# summary
if [ $success_count -eq $total_test_count ]; then
    echo -e "\033[32m\033[1mUnique count tests passed\033[0m"
else
    echo -e "\033[31m\033[1mFailed unique count tests\033[0m | Failed count: $((total_test_count-success_count))"
fi
test_result_hash_count=${#test_result_hash[@]}
unique_test_result_hash=($(echo "${test_result_hash[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))
unique_test_result_hash_count=${#unique_test_result_hash[@]}
if [ $unique_test_result_hash_count -eq 1 ] && [ $test_result_hash_count -eq $total_test_count ]; then
    echo -e "\033[32m\033[1mAll unique hash values are same\033[0m"
else
    echo -e "\033[31m\033[1mDifferent unique hash values\033[0m | Unique count: $unique_test_result_hash_count"
    for item in "${unique_test_result_hash[@]}"; do
        echo -e "$item"
    done
fi
