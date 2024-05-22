#!/bin/bash

min_num=0
max_num=100
cli_input_str="$@"
cli_input_arr=($cli_input_str)
re='^[0-9]+$'

help_msg () {
    echo "> This is a script to control Jetson Nano PWM fan with Linux device utility"
    echo "> Usage:  ./pwm_fan_jetson.sh <pwm_percentage>"
    echo "          $min_num ~ <pwm_percentage> ~ $max_num"
    exit 0
}

if [ -z "${cli_input_str}" ]; then
    echo "> No input argument provided."
    help_msg
    exit 0
fi

if [ $cli_input_str == "-h" ] || [ $cli_input_str == "--help" ]; then
    help_msg
    exit 0
fi

if ! [[ ${cli_input_arr[0]} =~ $re ]]; then
    echo "> Input: $@ | Use numbers between $min_num to $max_num"
    exit 1
fi

if [ ${cli_input_arr[0]} -lt $min_num ] || [ ${cli_input_arr[0]} -gt $max_num ]; then
    echo "> Input: $@ | Use numbers between $min_num to $max_num"
    exit 1
else
    pwm_value=$((cli_input_arr[0] * 255 / (max_num - min_num)))
    exec_cmd_arr=(sudo sh -c "echo $pwm_value > /sys/devices/pwm-fan/target_pwm")
    echo "> Executing -> ${exec_cmd_arr[@]}"
    "${exec_cmd_arr[@]}" || { echo "PWM fan inaccessable"; exit 1; }
    echo "> PWM fan speed set to ${cli_input_arr[0]}%"
fi
