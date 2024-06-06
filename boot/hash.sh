#!/bin/bash
#set -x # Uncomment to debug

# =========================
# >>>> CLI Interface
# -------------------------
help_msg () {
    echo ""
    echo "Usage:    bash hash.sh <dir_list_file> <initial_hash_value> [<hashed_file_list_storing_file>] [-p] [-r] [-tr] [-ts] [-tpm] [-v] [-h|--help]"
    echo ""
    echo "Necessary arguments:"
    echo "  <dir_list_file>:                    File containing list of directories to hash"
    echo "  <initial_hash_value>:               Initial hash value to start hashing chain by PCR SHA256 standard, should be 8x8 hex characters of 0~9 and A~F"
    echo ""
    echo "Optional arguments:"
    echo "  <hashed_file_list_storing_file>:    File to store hashed file list"
    echo "  -p:                                 (not supported yet) Enable UNIX Named Pipe to accelerate storage R/W"
    echo "  -r:                                 Enable RAMDisk to accelerate binary calling"
    echo "  -tr:                                Enable hash output trimming to remove \\n, space, filename, and so on"
    echo "  -ts:                                Enable timing how log it takes to hash each file"
    echo "  -tpm:                               Use TPM2.0 to hash files, default is shasum"
    echo "  -v:                                 Verbose mode"
    echo "  -h|--help:                          Display this help message"
    echo ""
    echo "Output variable: FINAL_HASH_VALUE     source this script to get the final hash value"
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
verbose=false
unix_named_pipe=false
ramdisk=false
trim=false
time_hash=false
tpm=false

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
temporary_hash_file="/tmp/hash" # file to store temporary hash value and file content for next hashing, extension will be added later
error_code=0

# =========================
# >>>> Error Codes Handling
# -------------------------
print_msg () {
    >&2 echo -e "> $error_message[$err_code]: $1"
}
print_error_msg () {
    case "$err_code" in
        # Section 0: CLI parsing
        # Offset: 00
        1)
            print_msg "[0.1] Failed to parse optional CLI arguments!"
            ;;
        2)
            print_msg "[0.2] Missing CLI argument: <dir_list_file>!"
            ;;
        3)
            print_msg "[0.2] Missing CLI argument: <initial_hash_value>!"
            ;;
        4)
            print_msg "[0.2] <dir_list_file> does not exist or is empty!"
            ;;
        5)
            print_msg "[0.2] <initial_hash_value> is not 64 characters long!"
            ;;
        6)
            print_msg "[0.2] <initial_hash_value> should consist of 0~9 and A~F only!"
            ;;
        7)
            print_msg "[0.3] Binary \"tpm2_hash\" not found!"
            ;;
        8)
            print_msg "[0.3] Binary \"shasum\" not found!"
            ;;
        9)
            print_msg "[0.3] Binary \"find\" not found!"
            ;;
        10)
            print_msg "[0.3] Binary \"cat\" not found!"
            ;;
        11)
            print_msg "[0.3] Binary \"echo\" not found!"
            ;;
        12)
            print_msg "[0.3] Binary \"xxd\" not found!"
            ;;
        # Section 1: File I/O
        # Offset: 20
        21)
            print_msg "[1.1] Failed to create temporary hash file!"
            ;;
        22)
            print_msg "[1.2] Failed to unmount existing RAMDisk!"
            ;;
        23)
            print_msg "[1.2] Failed to remove existing temporary folder for mounting RAMDisk!"
            ;;
        24)
            print_msg "[1.3] Failed to create temporary folder for mounting RAMDisk!"
            ;;
        25)
            print_msg "[1.3] Failed to change temporary folder's permission!"
            ;;
        26)
            print_msg "[1.3] Failed to mount RAMDisk!"
            ;;
        27)
            print_msg "[1.3] Failed to verify RAMDisk mounted!"
            ;;
        28)
            print_msg "[1.4] Failed to copy hash binary to RAMDisk!"
            ;;
        29)
            print_msg "[1.4] Copied hash binary doesn't work!"
            ;;
        30)
            print_msg "[1.4] Failed to copy ls to RAMDisk!"
            ;;
        31)
            print_msg "[1.4] Copied ls binary doesn't work!"
            ;;
        32)
            print_msg "[1.4] Failed to copy cat to RAMDisk!"
            ;;
        33)
            print_msg "[1.4] Copied cat binary doesn't work!"
            ;;
        34)
            print_msg "[1.4] Failed to copy echo to RAMDisk!"
            ;;
        35)
            print_msg "[1.4] Copied echo binary doesn't work!"
            ;;
        36)
            print_msg "[1.4] Failed to copy xxd to RAMDisk!"
            ;;
        37)
            print_msg "[1.4] Copied xxd binary doesn't work!"
            ;;
        # Section 2: Generating list of files and directories
        # Offset: 40
        41)
            print_msg "[2.1] Directory does not exist!"
            ;;
        42)
            print_msg "[2.2] Failed to remove duplicate files from array!"
            ;;
        43)
            print_msg "[2.4] Failed to write list of files and directories to temporary file!"
            ;;
        # Section 3: Hashing chain
        # Offset: 60
        # Section 4: I/O cleanup
        # Offset: 80
        81)
            print_msg "[4.2] Failed to clean up RAMDisk!"
            ;;
        *)
            echo -e "> $error_message[$err_code]: Unknown error!"
            ;;
    esac
}

# > 0. CLI parsing
echo "> Parsing CLI arguments ..."
cli_input_str="$@"
cli_input_arr=($cli_input_str)
dir_list_file=${cli_input_arr[0]}
initial_hash_value=${cli_input_arr[1]}
hashed_file_list_storing_file=${cli_input_arr[2]}
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
        echo "unix_named_pipe acceleration is not supported yet!"
        exit 1
    fi
    if [[ "${cli_input_arr[*]}" =~ "-r" ]]; then
        ramdisk=true
    fi
    if [[ "${cli_input_arr[*]}" =~ "-tr" ]]; then
        trim=true
    fi
    if [[ "${cli_input_arr[*]}" =~ "-ts" ]]; then
        time_hash=true
    fi
    if [[ "${cli_input_arr[*]}" =~ "-tpm" ]]; then
        tpm=true
    fi
    echo "verbose mode:                     $verbose"
    echo "unix_named_pipe acceleration:     $unix_named_pipe"
    echo "ramdisk acceleration:             $ramdisk"
    echo "trim hash output:                 $trim"
    echo "time hash process:                $time_hash"
    echo "use TPM for hashing:              $tpm"
    echo ""
fi
if [ $? -ne 0 ]; then
    err_code=1
fi
# *
# * 0.2 Check if required arguments are provided and valid
# *
echo "dir_list_file:                    $dir_list_file"
echo "initial_hash_value:               $initial_hash_value"
echo "hashed_file_list_storing_file:    $hashed_file_list_storing_file"
echo ""
if [[ $err_code -eq 0 ]]; then
    if [ -z "$dir_list_file" ]; then
        err_code=2
    fi
fi
if [[ $err_code -eq 0 ]]; then
    if [ -z "$initial_hash_value" ]; then
        err_code=3
    fi
fi
if [[ $err_code -eq 0 ]]; then
    if ! [ -s "$dir_list_file" ]; then
        err_code=4
    fi
fi
if [[ $err_code -eq 0 ]]; then
    if [ ${#initial_hash_value} -ne 64 ]; then
        err_code=5
        echo "> <initial_hash_value> is ${#initial_hash_value} characters long!"
    fi
fi
if [[ $err_code -eq 0 ]]; then
    if ! [[ $initial_hash_value =~ $hash_pattern ]]; then
        err_code=6
    fi
fi
if [[ $err_code -eq 0 ]]; then
    if [ -f "$hashed_file_list_storing_file" ]; then
        echo -e "> $warning_message: <hashed_file_list_storing_file> exists, moving it to ${hashed_file_list_storing_file}.bak ..."
        mv $hashed_file_list_storing_file "${hashed_file_list_storing_file}.bak"
    fi
fi
# *
# * 0.3 Check if binaries exist
# *
if [[ $err_code -eq 0 ]]; then
    if [ $tpm == true ]; then
        system_hash_bin=$(which tpm2_hash)
        if [ $? -ne 0 ]; then
            err_code=7
        fi
    else
        system_hash_bin=$(which shasum)
        if [ $? -ne 0 ]; then
            err_code=8
        fi
    fi
fi
if [[ $err_code -eq 0 ]]; then
    system_find=$(which find)
    if [ $? -ne 0 ]; then
        err_code=9
    fi
fi
if [[ $err_code -eq 0 ]]; then
    system_cat=$(which cat)
    if [ $? -ne 0 ]; then
        err_code=10
    fi
fi
if [[ $err_code -eq 0 ]]; then
    system_echo=$(which echo)
    if [ $? -ne 0 ]; then
        err_code=11
    fi
fi
if [[ $err_code -eq 0 ]]; then
    system_xxd=$(which xxd)
    if [ $? -ne 0 ]; then
        err_code=12
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
$system_echo "> Creating temporary files ..."
# *
# * 1.1 Create temporary hash file / named pipe
# *
if [[ $err_code -eq 0 ]]; then
    # this is necessary to avoid residual data in the file
    #if [ -f "$temporary_hash_file" ]; then # existence check doesn't work
    rm "$temporary_hash_file".*
    if [ $? -ne 0 ]; then
        echo -e "> $warning_message: Skipped removing existing temporary hash file!"
    fi
fi
if [[ $err_code -eq 0 ]]; then
    if [ $unix_named_pipe == true ]; then
        temporary_hash_file="${temporary_hash_file}.fifo"
    else
        temporary_hash_file="${temporary_hash_file}.tmp"
    fi
    if [ $unix_named_pipe == true ]; then
        mkfifo $temporary_hash_file
        exec 3<> $temporary_hash_file
    else
        mktemp "$temporary_hash_file.XXXXXXXX"
        temporary_hash_file=$(ls "${temporary_hash_file}"*)
    fi
    if [ $? -ne 0 ]; then
        echo "File: $temporary_hash_file"
        err_code=21
    fi
fi
# *
# * 1.2 Remove existing RAMDisk
# *
mbc_binary_ramdisk="/tmp/mbc_bin_ramdisk"
if [[ $err_code -eq 0 ]]; then
    if [ $ramdisk == true ]; then
        if grep -qs $mbc_binary_ramdisk /proc/mounts; then
            $system_echo "> RAMDisk already mounted!"
            $system_echo "> Removing existing RAMDisk ..."
            #sudo lsof -n $mbc_binary_ramdisk
            sudo umount $mbc_binary_ramdisk
            if [ $? -ne 0 ]; then
                err_code=22
            fi
            sudo rmdir $mbc_binary_ramdisk
            if [ $? -ne 0 ]; then
                err_code=23
            fi
        fi
    fi
fi
# *
# * 1.3 Mount binaries to RAMDisk
# *     - tpm2_hash: 4k
# *     - ls: 136k
# *     - cat: 36k
# *     - echo: 36k
# *   - create 5MB RAMDisk
# *
if [[ $err_code -eq 0 ]]; then
    if [ $ramdisk == true ]; then
        $system_echo "> Mounting binaries to RAMDisk ..."
        sudo mkdir -p $mbc_binary_ramdisk
        if [ $? -ne 0 ]; then
            err_code=24
        fi
        sudo chmod 777 $mbc_binary_ramdisk
        if [ $? -ne 0 ]; then
            err_code=25
        fi
        sudo mount -t tmpfs -o size=5M mbc_binary $mbc_binary_ramdisk
        if [ $? -ne 0 ]; then
            err_code=26
        fi
        mount | tail -n 1 | grep $mbc_binary_ramdisk
        if [ $? -ne 0 ]; then
            err_code=27
        fi
    fi
fi
# *
# * 1.4 Move binaries to RAMDisk
# *
if [[ $err_code -eq 0 ]]; then
    if [ $ramdisk == true ]; then
        $system_echo "> Copying binaries to RAMDisk ..."
        sudo cp $system_hash_bin $mbc_binary_ramdisk
        if [ $? -ne 0 ]; then
            err_code=28
        fi
        system_hash_bin_path=$(dirname $system_hash_bin)
        system_hash_bin=$mbc_binary_ramdisk/$(basename $system_hash_bin)
        $system_hash_bin --version > /dev/null
        if [ $? -ne 0 ]; then
            err_code=29
        fi

        sudo cp $system_find $mbc_binary_ramdisk
        if [ $? -ne 0 ]; then
            err_code=30
        fi
        system_find_path=$(dirname $system_find)
        system_find=$mbc_binary_ramdisk/$(basename $system_find)
        $system_find --version > /dev/null
        if [ $? -ne 0 ]; then
            err_code=31
        fi

        sudo cp $system_cat $mbc_binary_ramdisk
        if [ $? -ne 0 ]; then
            err_code=32
        fi
        system_cat_path=$(dirname $system_cat)
        system_cat=$mbc_binary_ramdisk/$(basename $system_cat)
        $system_cat --version > /dev/null
        if [ $? -ne 0 ]; then
            err_code=33
        fi

        sudo cp $system_echo $mbc_binary_ramdisk
        if [ $? -ne 0 ]; then
            err_code=34
        fi
        system_echo_path=$(dirname $system_echo)
        system_echo=$mbc_binary_ramdisk/$(basename $system_echo)
        $system_echo --version > /dev/null
        if [ $? -ne 0 ]; then
            err_code=35
        fi
        
        sudo cp $system_xxd $mbc_binary_ramdisk
        if [ $? -ne 0 ]; then
            err_code=36
        fi
        system_xxd_path=$(dirname $system_xxd)
        system_xxd=$mbc_binary_ramdisk/$(basename $system_xxd)
        $system_xxd --version 2>&1 /dev/null
        if [ $? -ne 0 ]; then
            err_code=37
        fi
    fi
fi

# *
# * 1.5 Section 1 ends
# *
if [[ $? -ne 0 ]] || [[ $err_code -ne 0 ]]; then
    print_error_msg
    exit 1
fi

# > 2. Generate list of files to hash
$system_echo "> Generating list of files to hash ..."
# *
# * 2.1 Find files in directories
# *
dir_list=($(cat "$dir_list_file"))
file_list=()
#IFS=$'\n' # https://stackoverflow.com/a/63969005/19138739
if [[ $err_code -eq 0 ]]; then
    for item in "${dir_list[@]}"; do
        #file_list+=($(find "$item" -type f | sed -e's/ /\\ /g'))
        file_list+=($(find "$item" -type f))
    done
    if [ ${#file_list[@]} -eq 0 ]; then
        err_code=41
        break
    fi
fi
#unset IFS
# *
# * 2.2 Remove duplicates
# *
$system_echo "> Removing duplicates ..."
if [[ $err_code -eq 0 ]]; then
    file_list=($(echo "${file_list[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' ')) # this line will breakup path with spaces
fi
if [[ $err_code -eq 0 ]]; then
    if [ ${#file_list[@]} -eq 0 ]; then
        err_code=42
    fi
fi
# *
# * 2.3 Verify files exist
# *
$system_echo "> Verifying files exist ..."
index_offset=0
if [[ $err_code -eq 0 ]]; then
    for index in "${!file_list[@]}"; do
        index=$((index - index_offset))
        if [ ! -f "${file_list[index]}" ]; then
            unset 'file_list[index]'
            index_offset=$((index_offset + 1))
            #$system_echo -e "$warning_message: ${file_list[index]} does not exist!" # Costly to print
        fi
        if [ -d "${file_list[index]}" ]; then
            unset 'file_list[index]'
            index_offset=$((index_offset + 1))
            $system_echo -e "$warning_message: ${file_list[index]} is a directory!"
        fi
    done
fi
# *
# * 2.4 Store file list
# *
$system_echo "> Storing file list ..."
if [[ $err_code -eq 0 ]]; then
    printf "%s\n" "${file_list[@]}" > "$hashed_file_list_storing_file"
    if [ $? -ne 0 ]; then
        err_code=43
    fi
fi
# *
# * 2.4 Section 2 ends
# *
if [[ $? -ne 0 ]] || [[ $err_code -ne 0 ]]; then
    print_error_msg
    exit 1
fi

# > 3. Perform hashing chain on files
#   - ref: https://www.youtube.com/watch?v=DFreHo3UCD0
#      - echo "hello" > file.fifo &
#      - cat < file.fifo
$system_echo "> Performing hashing chain on files ..."
hashing_chain () {
    $system_echo "> Hashing  ..."
    for index in "${!file_list[@]}"; do
        $system_echo -e "\e[1A\e[KHashing ${file_list[index]} ..."
        # Empty the temporary hash file
        > $temporary_hash_file
        # Add initial hash value
        $system_echo "$initial_hash_value" >> $temporary_hash_file
        # Add path of file
        $system_echo "${file_list[index]}" >> $temporary_hash_file
        # Add file content
        $system_cat ${file_list[index]} >> $temporary_hash_file
        #if [ $unix_named_pipe == true ]; then
        #    # get file line count
        #    #
        #    :
        #fi
        # Hash the file
        if [ $trim == true ]; then
            if [ $tpm == true ]; then
                initial_hash_value="$($system_hash_bin -C o -g sha256 $temporary_hash_file | tr -d '\n')"
            else
                initial_hash_value="$($system_hash_bin -a 256 $temporary_hash_file)"
                initial_hash_value="${initial_hash_value//$temporary_hash_file/}"
            fi
        else
            if [ $tpm == true ]; then
                initial_hash_value="$($system_hash_bin -C o -g sha256 $temporary_hash_file)"
            else
                initial_hash_value="$($system_hash_bin -a 256 $temporary_hash_file)"
                initial_hash_value="${initial_hash_value//$temporary_hash_file/}"
            fi
        fi
    done
    if [ $tpm == true ]; then
        FINAL_HASH_VALUE="$(echo $initial_hash_value | tr -d '\n' | $system_xxd -p | tr -d '\n' | tr -d ' ')"
        export FINAL_HASH_VALUE="$FINAL_HASH_VALUE"
    else
        export FINAL_HASH_VALUE="${initial_hash_value//$temporary_hash_file/}"
    fi
}
if [ $time_hash == true ]; then
    time hashing_chain
else
    hashing_chain
fi
echo $FINAL_HASH_VALUE

# > 4. Remove I/O files
$system_echo "> Removing temporary files ..."
# *
# * 4.1 Remove temporary hash file
# *
if [[ $err_code -eq 0 ]]; then
    #if [ -f "$temporary_hash_file" ]; then # existence check doesn't work
    #rm $temporary_hash_file
    echo -e "Temporary hash file is not removed! This is still in testing phase."
    if [ $? -ne 0 ]; then
        echo -e "> $warning_message: Skipped removing temporary hash file!"
    fi
fi
# *
# * 4.2 Unmount binaries from RAMDisk
# *   - experiments show that it is impossible to unmount the RAMDisk with the same process creating it
# *   - so, only remove the files in the RAMDisk
# *
if [[ $err_code -eq 0 ]]; then
    if [ $ramdisk == true ]; then
        $system_echo "> Removing binaries from RAMDisk ..."
        sudo rm -rf $mbc_binary_ramdisk/*
        if [ $? -ne 0 ]; then
            err_code=81
        fi
        system_hash_bin=$system_hash_bin_path/$(basename $system_hash_bin)
        system_find=$system_find_path/$(basename $system_find)
        system_cat=$system_cat_path/$(basename $system_cat)
        system_echo=$system_echo_path/$(basename $system_echo)
        system_xxd=$system_xxd_path/$(basename $system_xxd)
        $system_echo "It is currently impossible to unmount the RAMDisk with the same process creating it."
        $system_echo "To unmout the RAMDisk manually, run the following command: (it would be unmounted next time this script is [auto-]executed)"
        $system_echo "    sudo umount $mbc_binary_ramdisk && sudo rmdir $mbc_binary_ramdisk"
    fi
fi
# *
# * 4.3 Section 4 ends
# *
if [[ $? -ne 0 ]] || [[ $err_code -ne 0 ]]; then
    print_error_msg
    exit 1
fi

$system_echo "Execution ends successfully!"
