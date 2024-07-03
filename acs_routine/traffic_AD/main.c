/* @file main
 * @brief Main function for ACS attestation log DB anomaly detection,
 *        No error handling is implemented in this version.
 * @authoer belongtothenight / Da-Chuan Chen / 2024-07-03
 */

#include <stdio.h>
#include <stdlib.h>

void print_help_message (void) {
    printf("Usage:\n");
    printf("  ./traffic_AD -H <hostname> -I <interface> -i <interval> [-v]\n");
    printf("  ./traffic_AD -h\n");
    printf("Options:\n");
    printf("  -H, --hostname <hostname>         Set the hostname to monitor (e.g. 192.168.0.1)\n");
    printf("  -I, --pcap-interface <interface>  Set the interface to capture traffic (e.g. wlan0)\n");
    printf("  -i, --interval <interval>         Set the interval of measurement in seconds (e.g. 30)\n");
    printf("  -v, --verbose                     Verbose output\n");
    printf("  -h, --help                        Show this help message and exit\n");
    printf("Output format:\n");
    printf("  <result_num>:<timestamp>:<hostname/IP>:<vector_deviation>:<result_info>\n");
    printf("Description:\n");
    printf("  This program will monitor the traffic from specific IP in its first execution interval, and compare with following intervals to check for anomaly situation.\n");
    return;
}

int main (int argc, char *argv[]) {
    printf("Hello, World!\n");
    return;
}
