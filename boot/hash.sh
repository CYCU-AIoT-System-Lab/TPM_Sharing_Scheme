#!/bin/bash
#set -x # Uncomment to debug

# =========================
# >>>> CLI Interface
# -------------------------
help_msg () {
    echo ""
    echo "Usage:    bash hash.sh <dir_list_file> <initial_hash_value> [<hashed_file_list_storing_file>] [-v] [-p] [-r] [-h|--help]"
    echo "  <dir_list_file>:                    File containing list of directories to hash"
    echo "  <initial_hash_value>:               Initial hash value to start hashing chain by PCR SHA256 standard, should be 8x8 hex characters of 0~9 and A~F"
    echo "  <hashed_file_list_storing_file>:    (optional) File to store hashed file list"
    echo "  -v:                                 Verbose mode"
    echo "  -p:                                 Enable UNIX Named Pipe to accelerate storage R/W"
    echo "  -r:                                 Enable RAMDisk to accelerate binary calling"
    echo "  -h|--help:                          Display this help message"
    echo ""
    echo "Example:  bash hash.sh -h"
    echo "          bash hash.sh dir_list.txt 1234567890ABCDEF000000000000000000000000000000000000000000000000"
    echo "          bash hash.sh dir_list.txt 1234567890ABCDEF000000000000000000000000000000000000000000000000 hashed_file_list.txt"
    echo "          bash hash.sh dir_list.txt 1234567890ABCDEF000000000000000000000000000000000000000000000000 hashed_file_list.txt -v -p -r"
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
temporary_hash_file="/tmp/hash" # file to store temporary hash value and file content for next hashing, extension will be added later
error_code=0

# =========================
# >>>> Error Codes Handling
# -------------------------
print_msg () {
    echo -e "> $error_message[$err_code]: $1"
}
print_error_msg () {
    case "$err_code" in
        # Section 0: CLI parsing
        # Offset: 00
        1)
            print_msg "Failed to parse optional CLI arguments!"
            ;;
        2)
            print_msg "Binary \"tpm2_hash\" not found!"
            ;;
        3)
            print_msg "Binary \"ls\" not found!"
            ;;
        4)
            print_msg "Binary \"cat\" not found!"
            ;;
        5)
            print_msg "Binary \"echo\" not found!"
            ;;
        6)
            print_msg "Missing <dir_list_file>!"
            ;;
        7)
            print_msg "Missing <initial_hash_value>!"
            ;;
        8)
            print_msg "<dir_list_file> does not exist or is empty!"
            ;;
        9)
            print_msg "<initial_hash_value> is not 64 characters long!"
            ;;
        10)
            print_msg "<initial_hash_value> should consist of 0~9 and A~F only!"
            ;;
        # Section 1: File I/O
        # Offset: 20
        21)
            print_msg "Failed to create temporary hash file!"
            ;;
        22)
            print_msg "Failed to create hashed file list storing file!"
            ;;
        # Section 2: Generating list of files and directories
        # Offset: 40
        41)
            print_msg "Directory does not exist!"
            ;;
        42)
            print_msg "Failed to list files in directory!"
            ;;
        43)
            print_msg "Failed to remove duplicates in file list!"
            ;;
        44)
            print_msg "Failed to remove duplicates in directory list!"
            ;;
        45)
            print_msg "Failed to check if files exist!"
            ;;
        46)
            print_msg "Failed to store list of files and dirs to hash in a file!"
            ;;
        *)
            echo -e "> $error_message[$err_code]: Unknown error!"
            ;;
    esac
}

# > 0. CLI parsing
$system_echo "> Parsing CLI arguments ..."
err_code_offset=00
cli_input_str="$@"
cli_input_arr=($cli_input_str)
dir_list_file=${cli_input_arr[0]}
initial_hash_value=${cli_input_arr[1]}
hashed_file_list_storing_file=${cli_input_arr[2]}
verbose=false
unix_named_pipe=false
ramdisk=false
# *
# * 0.1 Process optional arguments
# *
if [ $? -eq 0 ]; then
    if [[ "${cli_input_arr[*]}" =~ "-h" ]] || [[ "${cli_input_arr[*]}" =~ "--help" ]]; then
        help_msg
        exit 0
    fi
    if [[ "${cli_input_arr[*]}" =~ "-v" ]]; then
        verbose=true
    fi
    if [[ "${cli_input_arr[*]}" =~ "-p" ]]; then
        unix_named_pipe=true
    fi
    if [[ "${cli_input_arr[*]}" =~ "-r" ]]; then
        ramdisk=true
    fi
    $system_echo "verbose mode:                     $verbose"
    $system_echo "unix_named_pipe acceleration:     $unix_named_pipe"
    $system_echo "ramdisk acceleration:             $ramdisk"
fi
if [ $? -ne 0 ]; then
    err_code=$((err_code_offset+1))
fi
# *
# * 0.2 Check if binaries exist
# *
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if [ -z "$system_tpm2_hash" ]; then
        err_code=$((err_code_offset+2))
    fi
fi
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if [ -z "$system_ls" ]; then
        err_code=$((err_code_offset+3))
    fi
fi
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if [ -z "$system_cat" ]; then
        err_code=$((err_code_offset+4))
    fi
fi
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if [ -z "$system_echo" ]; then
        err_code=$((err_code_offset+5))
    fi
fi
# *
# * 0.3 Check if required arguments are provided and valid
# *
$system_echo "dir_list_file:                    $dir_list_file"
$system_echo "initial_hash_value:               $initial_hash_value"
$system_echo "hashed_file_list_storing_file:    $hashed_file_list_storing_file"
$system_echo ""
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if [ -z "$dir_list_file" ]; then
        err_code=$((err_code_offset+6))
    fi
fi
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if [ -z "$initial_hash_value" ]; then
        err_code=$((err_code_offset+7))
    fi
fi
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if ! [ -s "$dir_list_file" ]; then
        err_code=$((err_code_offset+8))
    fi
fi
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if [ ${#initial_hash_value} -ne 64 ]; then
        err_code=$((err_code_offset+9))
        $system_echo "> <initial_hash_value> is ${#initial_hash_value} characters long!"
    fi
fi
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if ! [[ $initial_hash_value =~ $hash_pattern ]]; then
        err_code=$((err_code_offset+10))
    fi
fi
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if [ -f "$hashed_file_list_storing_file" ]; then
        $system_echo -e "> $warning_message: <hashed_file_list_storing_file> exists, moving it to ${hashed_file_list_storing_file}.bak ..."
        mv $hashed_file_list_storing_file "${hashed_file_list_storing_file}.bak"
    fi
fi
# *
# * 0.4 Section CLI arguments parsing ends
# *
if [[ $? -ne 0 ]] || [[ $err_code -ne 0 ]]; then
    print_error_msg
    help_msg
    exit 1
fi

# > 1. File I/O
# *  - Create temporary hash file
# *  - Mout binaries to RAMDisk (4K+49K)
# *     - tpm2_hash: 4k
# *     - ls: 136k
# *     - cat: 36k
# *     - echo: 36k
$system_echo "> Creating temporary files ..."
err_code_offset=$((err_code_offset+20)) # 20
# *
# * 1.1 Create temporary hash file
# *
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    #if [ -f "$temporary_hash_file" ]; then # existence check doesn't work
    rm "$temporary_hash_file".*
    if [ $? -ne 0 ]; then
        echo -e "> $warning_message: Skipped removing existing temporary hash file!"
    fi
fi
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    if [ $unix_named_pipe == true ]; then
        temporary_hash_file="${temporary_hash_file}.fifo"
    else
        temporary_hash_file="${temporary_hash_file}.tmp.XXXXXXXX"
    fi
    if [ $unix_named_pipe == true ]; then
        mkfifo $temporary_hash_file
    else
        mktemp $temporary_hash_file
    fi
    if [ $? -ne 0 ]; then
        echo "File: $temporary_hash_file"
        err_code=$((err_code_offset+1))
    fi
fi
# *
# * 1.2 Mount binaries to RAMDisk
# *

# *
# * 1.3 Section 1 ends
# *
if [[ $? -ne 0 ]] || [[ $err_code -ne 0 ]]; then
    print_error_msg
    exit 1
fi

# > 2. Generate list of files to hash
err_code_offset=$((err_code_offset+20)) # 40
$system_echo "> Generating list of files to hash ..."
# *
# * 2.1 Check if directories exist
# *
dir_list=($(cat $dir_list_file))
file_list=()
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    for dir in "${dir_list[@]}"; do
        if [ ! -d "$dir" ]; then
            err_code=$((err_code_offset+1))
            echo "Not existed dir: $dir"
        fi
        # $system_ls -AR $dir
        file_list+=($($system_ls -AR $dir))
    done
    if [ $? -ne 0 ]; then
        err_code=$((err_code_offset+1))
    fi
fi
# *
# * 2.2 Process file list
# *     - Move directories to dir_list
# *     - Convert relative path to absolute path
# *
file_dir="$script_path:"
index_offset=0
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
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
    if [ $? -ne 0 ]; then
        err_code=$((err_code_offset+2))
    fi
fi
# *
# * 2.3 Remove duplicates of files and dirs
# *    - ref: https://stackoverflow.com/questions/13648410/how-can-i-get-unique-values-from-an-array-in-bash
# *
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
    file_list=($($system_echo "${file_list[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))
    if [ $? -ne 0 ]; then
        err_code=$((err_code_offset+3))
    fi
    dir_list=($($system_echo "${dir_list[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))
    if [ $? -ne 0 ]; then
        err_code=$((err_code_offset+4))
    fi
fi
# *
# * 2.4 Check if files exist, also removing included directories
# *
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
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
    if [ $? -ne 0 ]; then
        err_code=$((err_code_offset+5))
    fi
fi
# *
# * 2.5 Store list of files and dirs to hash in a file
# *
if [[ $? -eq 0 ]] && [[ $err_code -eq 0 ]]; then
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
    if [ $? -ne 0 ]; then
        err_code=$((err_code_offset+6))
    fi
fi
# *
# * 2.6 Section 2 ends
# *
if [[ $? -ne 0 ]] || [[ $err_code -ne 0 ]]; then
    print_error_msg
    exit 1
fi

# > 3. Perform hashing chain on files
err_code_offset=$((err_code_offset+20)) # 60
#   - ref: https://www.youtube.com/watch?v=DFreHo3UCD0
#      - echo "hello" > file.fifo &
#      - cat < file.fifo

# > 4. Unmount binaries from RAMDisk
err_code_offset=$((err_code_offset+20)) # 80

$system_echo "end of script"
