#!/bin/bash
#set -x

# =========================
# >>>> CLI Interface
# -------------------------
help_msg () {
    echo ""
    echo "Usage:    bash hash.sh <dir_list_file> <initial_hash_value> [<hashed_file_list_storing_file>] [-v] [-h|--help]"
    echo "  <dir_list_file>:                    File containing list of directories to hash"
    echo "  <initial_hash_value>:               Initial hash value to start hashing chain by PCR SHA256 standard, should be 8x8 hex characters of 0~9 and A~F"
    echo "  <hashed_file_list_storing_file>:    (optional) File to store hashed file list"
    echo "  -v:                                 Verbose mode"
    echo "  -h|--help:                          Display this help message"
    echo ""
    echo "Example:  bash hash.sh -h"
    echo "          bash hash.sh dir_list.txt 1234567890ABCDEF000000000000000000000000000000000000000000000000"
    echo "          bash hash.sh dir_list.txt 1234567890ABCDEF000000000000000000000000000000000000000000000000 hashed_file_list.txt"
    echo "          bash hash.sh dir_list.txt 1234567890ABCDEF000000000000000000000000000000000000000000000000 hashed_file_list.txt -v"
}

# =========================
# >>>> Purpose
# -------------------------
# Take a file containing a list of directories to hash, and initial hash value as input
# Using tpm2tss hashing binary to perform hashing chain on files from directories

# =========================
# >>>> Ajustable Parameters
# -------------------------

# =========================
# >>>> Static Parameters
# -------------------------
error_message="\033[31m\033[1mError\033[0m"
warning_message="\033[33m\033[1mWarning\033[0m"
hash_pattern='^[0-9A-Fa-f]+$'
script=$(realpath "$0")
script_path=$(dirname "$script")
system_tpm2_hash=$(which tpm2_hash)
system_ls=$(which ls)
system_cat=$(which cat)
system_echo=$(which echo)
if [ -z "$system_tpm2_hash" ]; then
    $system_echo -e "> $error_message: Binary "tpm2_hash" not found!"
    exit 1 # skip for now
fi
if [ -z "$system_ls" ]; then
    $system_echo -e "> $error_message: Binary "ls" not found!"
    exit 1
fi
if [ -z "$system_cat" ]; then
    $system_echo -e "> $error_message: Binary "cat" not found!"
    exit 1
fi
if [ -z "$system_echo" ]; then
    $system_echo -e "> $error_message: Binary "echo" not found!"
    exit 1
fi

# > 0. CLI parsing
$system_echo "> Parsing CLI arguments ..."
cli_input_str="$@"
cli_input_arr=($cli_input_str)
dir_list_file=${cli_input_arr[0]}
initial_hash_value=${cli_input_arr[1]}
hashed_file_list_storing_file=${cli_input_arr[2]}
verbose=false
if [ ${cli_input_arr[3]} == "-v" ]; then
    verbose=true
fi
$system_echo "dir_list_file:                    $dir_list_file"
$system_echo "initial_hash_value:               $initial_hash_value"
$system_echo "hashed_file_list_storing_file:    $hashed_file_list_storing_file"
$system_echo ""

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    help_msg
    exit 0
fi
if [ ${#cli_input_arr[@]} -ne 2 ] && [ ${#cli_input_arr[@]} -ne 3 ] && [ ${#cli_input_arr[@]} -ne 4 ]; then
    $system_echo -e "> $error_message: Incorrect number of arguments!"
    help_msg
    exit 1
fi
if [ -z "$dir_list_file" ]; then
    $system_echo -e "> $error_message: Missing dir_list_file!"
    help_msg
    exit 1
fi
if [ -z "$initial_hash_value" ]; then
    $system_echo -e "> $error_message: Missing initial_hash_value!"
    help_msg
    exit 1
fi
if ! [ -s "$dir_list_file" ]; then
    $system_echo -e "> $error_message: <dir_list_file> does not exist or is empty!"
    help_msg
    exit 1
fi
if [ ${#initial_hash_value} -ne 64 ]; then
    $system_echo -e "> $error_message: <initial_hash_value> should be 64 characters long, but it is ${#initial_hash_value}!"
    help_msg
    exit 1
fi
if ! [[ $initial_hash_value =~ $hash_pattern ]]; then
    $system_echo -e "> $error_message: <initial_hash_value> should consist of 0~9 and A~F!"
    help_msg
    exit 1
fi
if [ -f "$hashed_file_list_storing_file" ]; then
    $system_echo -e "> $warning_message: <hashed_file_list_storing_file> exists, moving it to ${hashed_file_list_storing_file}.bak ..."
    mv $hashed_file_list_storing_file "${hashed_file_list_storing_file}.bak"
fi

# > 1. Mout binaries to RAMDisk (4K+49K)
# *  - tpm2_hash: 4k
# *  - ls: 136k
# *  - cat: 36k
# *  - echo: 36k

# > 2. Generate list of files to hash
$system_echo "> Generating list of files to hash ..."
dir_list=($(cat $dir_list_file))
file_list=()
# *
# * 1.1 Check if directories exist
# *
for dir in "${dir_list[@]}"; do
    if [ ! -d "$dir" ]; then
        $system_echo -e "> $error_message: Directory $dir does not exist!"
        exit 1
    fi
    # $system_ls -AR $dir
    file_list+=($($system_ls -AR $dir))
done
# *
# * 1.2 Process file list
# *     - Move directories to dir_list
# *     - Convert relative path to absolute path
# *
file_dir="$script_path:"
index_offset=0
for index in "${!file_list[@]}"; do
    # if is path, start with /
    #$system_echo ""
    #$system_echo ${file_list[@]}
    index=$((index-index_offset))
    if [[ "${file_list[index]}" = /* ]]; then
        temp="${file_list[index]}"
        temp_dir="${temp::-1}"
        if [ $verbose == true ]; then
            $system_echo "dir:  $temp_dir"
        fi
        if ! [[ " ${dir_list[@]} " =~ " $temp_dir " ]]; then
            dir_list+=($temp_dir)
        fi
        file_dir="$temp"
        file_list=(${file_list[@]/$temp})
        index_offset=$((index_offset+1))
    else
        file_list[index]="${file_dir::-1}/${file_list[index]}"
        if [ $verbose == true ]; then
            $system_echo "file: ${file_list[index]}"
        fi
    fi
done
# *
# * 1.3 Remove duplicates of files and dirs
# *    - ref: https://stackoverflow.com/questions/13648410/how-can-i-get-unique-values-from-an-array-in-bash
# *
file_list=($($system_echo "${file_list[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))
dir_list=($($system_echo "${dir_list[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))
# *
# * 1.4 Check if files exist, also removing included directories
# *
for index in "${!file_list[@]}"; do
    if [ ! -f "${file_list[index]}" ]; then
        #file_list=(${file_list[@]/${file_list[index]}}) # can't remove element by value cause it would also remove other path containing the value
        unset -v 'file_list[index]'
        if [ -d "${file_list[index]}" ]; then
            if [ $verbose == true ]; then
                $system_echo -e "$warning_message: removed directory ${file_list[index]}"
            fi
        else
            if ! [ -z "${file_list[index]}" ]; then
                $system_echo -e "$warning_message: removed ${file_list[index]}"
            fi
        fi
    fi
done
$system_echo "> Number of files to hash: ${#file_list[@]}"
$system_echo "> Number of directories: ${#dir_list[@]}"
if ! [ -z $hashed_file_list_storing_file ]; then
    $system_echo "Files:" > $hashed_file_list_storing_file
    for file in "${file_list[@]}"; do
        $system_echo "$file" >> $hashed_file_list_storing_file
    done
    $system_echo -e "\nDirectory:" >> $hashed_file_list_storing_file
    for dir in "${dir_list[@]}"; do
        $system_echo "$dir" >> $hashed_file_list_storing_file
    done
    $system_echo "> Files to hash are stored in $hashed_file_list_storing_file"
fi

# > 3. Perform hashing chain on files
# > 4. Unmount binaries from RAMDisk

$system_echo "end of script"
