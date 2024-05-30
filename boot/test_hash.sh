#!/bin/bash
# This script is used to test the consistency of hashing script

test_total_count=100

test_default=true
test_pipe=false
test_ramdisk=false
test_trim=true

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
    echo -e "Unique count: ${#unique_result_arr[@]}"
    for index in "${!unique_result_arr[@]}"; do
        temp_item=${unique_result_arr[$index]}
        temp_len=${#unique_result_arr[$index]}
        temp_diff=$((ideal_len - temp_len))
        temp_cnt=$(grep -o "$temp_item" <<< "${result_arr[@]}" | wc -l)
        echo -e "Cnt $temp_cnt \tlen $temp_len \tlen_diff $temp_diff \t$temp_item"
    done
}

# test defualt
if [ $test_default == true ]; then
    echo -e "Testing default hash.sh"
    test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v"
fi

# test with pipe
if [ $test_pipe == true ]; then
    echo "Testing hash.sh with pipe"
    execute_hash_with_args "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-p"
fi

# test with ramdisk
if [ $test_ramdisk == true ]; then
    echo "Testing hash.sh with ramdisk"
    test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-r"
fi

# test with trim
if [ $test_trim == true ]; then
    echo "Testing hash.sh with trim"
    test_command_and_list_unique_cnt "./hash.sh" "dir_list.txt" "1234567890ABCDEF000000000000000000000000000000000000000000000000" "hashed_file_list.txt" "-v" "-tr"
fi
