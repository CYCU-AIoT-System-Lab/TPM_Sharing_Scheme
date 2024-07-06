/* @file main
 * @brief Main function for ACS attestation log DB anomaly detection,
 *        No error handling is implemented in this version.
 * @authoer belongtothenight / Da-Chuan Chen / 2024-07-03
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "libtrace.h"

#include "../lib/lib_signal_handler.h"

#define LIBTRACE_ERRNO      1000
#define NANOSEC_CONVERT     1000000000
#define NANOSEC_CONVERT2    2000000000

/* general */
volatile int        exit_flag = false;      /* Global exit flag modify by signal handler */
volatile int        filter_flag = false;    /* Global flag to filter out monitoring traffic */
volatile int        filter_applied = false; /* Global flag to prevent filter being re-applied */
long int            window_ws = 0;
long int            window_wc = 0;
long unsigned int   old_f2 = 0;             /* F2 score of previous calculation */
long unsigned int   new_f2 = 0;
long unsigned int   dif_f2 = 0;

/* CLI */
const char         *hostname = NULL;        /* Monitoring IPv4 address */
const char         *interface = NULL;       /* libtrace pcap interface */
long int            window_size = 0;
long int            window_count = 0;
long unsigned int   threshold_f2 = 0;
volatile bool       verbose = false;        /* Verbose mode */

/* timespec */
time_t              previous_s = 0;         /* store time when last window ends */
long int            previous_ns = 0;
long int            elapse_ns = 0;          /* duration kept in nano-second */

/* packet */
long int            vec_p_win = 0;          /* window pointer, used as index of vec */
long int           *packet_cnt_vec = NULL;  /* packet count vector for initial interval */

void print_help_message (void) {
    printf("Usage:\n");
    printf("  ./traffic_AD -H <hostname> -i <interface> -s <window_size> -c <window_count> -t <f2_threshold> [-v]\n");
    printf("  ./traffic_AD -h\n");
    printf("Options:\n");
    printf("  -H, --hostname <hostname>         Set the hostname to monitor (e.g. 192.168.0.1)\n");
    printf("  -i, --pcap-interface <interface>  Set the interface to capture traffic (e.g. wlan0)\n");
    printf("  -s, --window-size <number>        Window size in nano-second, power of 2\n");
    printf("  -c, --window-count <number>       Window count in a interval, power of 2\n");
    printf("  -t, --f2-threshold <number>       F2 score to trigger packet filtering\n");
    printf("  -v, --verbose                     Verbose output\n");
    printf("  -h, --help                        Show this help message and exit\n");
    printf("Output format:\n");
    printf("  <result_num>:<timestamp>:<hostname/IP>:<vector_deviation>:<result_info>\n");
    printf("Description:\n");
    printf("  This program will use previous interval monitored traffic to compare with the next one with F2 value to check for anomaly situation. Note that the first interval output is always wrong. Whenever traffic anomaly is detected or received SIGUSR1, this program applies netfilter with iptables to block all traffic from hostname.");
    printf("Size:\n");
    printf("  ws: window size\n");
    printf("  wc: window count\n");
    printf("  |-----------------interval------------------|\n");
    printf("  |-----------------2^ws*2^wc-----------------|\n");
    printf("  |--window--|--window--|--window--|--window--|\n");
    printf("  |---2^ws---|---2^ws---|---2^ws---|---2^ws---|\n");
    return;
}

/* First approach is using libtrace BPF filtering API, but
 * it's not supported in the current configuration which
 * displayed in `man trace`
 *
 * Second approach is using `iptables` to access netfilters
 * No need to open with pipe since it does not need to be
 * persist throughout multiple commands
 */
void apply_filter (void) {
    char command[100];
    int command_status;
    snprintf(command, sizeof(command), "sudo iptables -A INPUT -s %s -j DROP", hostname);
    command_status = system(command);
    printf("Received %d after applying filter with command: \t%s\n", command_status, command);
    printf("To see applied filter chains:               \tsudo iptables -L -n -v\n");
    filter_applied = true;
    return;
}

void remove_filter (void) {
    char command[100];
    int command_status;
    snprintf(command, sizeof(command), "sudo iptables -D INPUT -s %s -j DROP", hostname);
    command_status = system(command);
    printf("Received %d after applying filter with command: \t%s\n", command_status, command);
    printf("To see applied filter chains:               \tsudo iptables -L -n -v\n");
    filter_applied = false;
    return;
}

void zero_vec (long int *vec) {
    for (long int i=0; i<window_wc; ++i) {
        vec[i] = 0;
    }
}

void init_vec (long int *vec) {
    for (long int i=0; i<window_wc; ++i) {
        printf("%ld\r", i); // somehow required so that initial vec won't have weird value
        vec[i] = 1;
        vec[i] = 0;
    }
}

void print_vec (long int *vec) {
    printf("[");
    for (long int i=0; i<window_wc; ++i) {
        printf("%ld ", vec[i]);
    }
    printf("]\n");
}

void f2_vec (long int *vec) {
    new_f2 = 0;
    for (long int i=0; i<window_wc; ++i) {
        new_f2 += (long unsigned int)(vec[i] * vec[i]);
    }
}

void f2_diff_abs (long unsigned int a, long unsigned int b) {
    if (a>b) dif_f2 = a-b;
    else if (a<b) dif_f2 = b-a;
    else dif_f2 = 0;
}

/* filter out packets came from monitoring hostname */
static inline bool filter_ip(struct sockaddr *ip) {
    char str[40];
    if (ip->sa_family==AF_INET) {
        struct sockaddr_in *v4 = (struct sockaddr_in *)ip;
        inet_ntop(AF_INET, &(v4->sin_addr), str, sizeof(str));
        //if (verbose) printf("%s\n", str);
        if (strncmp(str, hostname, 15)==0) {
            return true;
        }
    }
    return false;
}

/*
 * Use as much bit shit >> << as possible to optimize for / and %
 */
static void per_packet (libtrace_packet_t *packet) {
    struct sockaddr_storage addr;
    struct sockaddr *addr_ptr;
    bool monitored_host = false;

    addr_ptr = trace_get_source_address(packet, (struct sockaddr *)&addr);
    if (addr_ptr == NULL) ;//printf("NULL");
    else monitored_host = filter_ip(addr_ptr);

    if (monitored_host == true) {
    //if (monitored_host == true) {
        /* calculate current time and previous time
         * difference as elapse time and store as 
         * nano-second
         *
         * if minusing to negative result, elapse time
         * calculation will still work
         */
        struct timespec ts = trace_get_timespec(packet);
        //time_t tmp_s = ts.tv_sec - previous_s;
        //time_t tmp_ns = ts.tv_nsec - previous_ns;
        //if (tmp_ns < 0) {
        //    tmp_s -= 1;
        //    tmp_ns += NANOSEC_CONVERT;
        //}
        //elapse_ns = (tmp_s) * NANOSEC_CONVERT + (tmp_ns);
        elapse_ns = (ts.tv_sec - previous_s) * NANOSEC_CONVERT + (ts.tv_nsec - previous_ns);
        //printf("Elapse_ns: %ld\n", elapse_ns);
        //printf("Window_ns: %ld\n", window_ws);

        /* check for window switch */
        if (elapse_ns >= window_ws) {
            /* calculate vec pointer increment */
            long int vec_p_inc = elapse_ns >> window_size;
            /* increment vec pointer */
            vec_p_win += vec_p_inc;
            if (verbose) printf("vec_p_win: %ld vec_p_inc: %ld\n", vec_p_win, vec_p_inc);
            /* update previous time 
             *
             * can't use the time from packet since it is not the
             * last nano-second of previous window
             *
             * since packets are more likely to come in less than 
             * 2 seconds interval, do special case processing for 
             * this interval
             * */
            previous_ns += vec_p_inc * window_ws;
            if (previous_ns >= NANOSEC_CONVERT) {
                // 1s ~ 2s
                previous_s++;
                previous_ns -= NANOSEC_CONVERT;
                if (previous_ns < NANOSEC_CONVERT2) {
                    // > 2s
                    long int previous_inc = previous_ns / NANOSEC_CONVERT;
                    previous_s += previous_inc;
                    previous_ns -= previous_inc * NANOSEC_CONVERT;
                }
            } // 0s ~ 1s do nothing

            /* check for interval switch */
            if (vec_p_win >= window_wc) {
                /* new f2 score cal */
                f2_vec(packet_cnt_vec);
                f2_diff_abs(old_f2, new_f2);
                if (verbose) printf("F2 old: %lu new: %lu absdiff: %lu\n", old_f2, new_f2, dif_f2);
                if (dif_f2 > threshold_f2) {
                    printf("d\n");
                    printf("1:%ld.%ld:%s:%lu:diviation_too_large\n", ts.tv_sec, ts.tv_nsec, hostname, dif_f2);
                    filter_flag = true; // activate packet filtering
                    printf("filtering hostname\n");
                }
                old_f2 = new_f2;
                /* update window pointer */
                vec_p_win -= window_wc * (vec_p_win >> window_count);
                if (verbose) print_vec(packet_cnt_vec);
                zero_vec(packet_cnt_vec);
                if (verbose) printf("\n");
            }
        }
        packet_cnt_vec[vec_p_win]++;
    }

    if (filter_flag==true) apply_filter();
}

int main (int argc, char *argv[]) {
    /* register signal handler */
    register_all_signal_handlers();
    if (errno != 0) {
        perror("signal");
    }

    /* param */
                        errno = 0;          /* error number */
    int                 i = 0;              /* iterator */
    char               *endptr = NULL;      /* string to long int conversion pointer */
    int                 base = 10;          /* string to long int conversion base */
    libtrace_t         *trace = NULL;       /* libtrace */
    libtrace_packet_t  *packet = NULL;      /* libtrace packet */

    /* CLI parsing */
    for (i=1; (i<argc) && (errno==0); i++) {
        if ((strcmp(argv[i], "-H")==0) || (strcmp(argv[i], "--hostname")==0)) {
            i++;
            if (i<argc) hostname = argv[i];
            else errno = EINVAL;
        } else if ((strcmp(argv[i], "-i")==0) || (strcmp(argv[i], "--pcap-interface")==0)) {
            i++;
            if (i<argc) interface = argv[i];
            else errno = EINVAL;
        } else if ((strcmp(argv[i], "-s")==0) || (strcmp(argv[i], "--window-size")==0)) {
            i++;
            if (i<argc) {
                window_size = strtol(argv[i], &endptr, base);
                if (errno != 0) {
                    perror("strtod");
                    errno = EXIT_FAILURE;
                }
                if (endptr == argv[i]) {
                    fprintf(stderr, "No digits were found\n");
                    errno = EXIT_FAILURE;
                }
            } else errno = EINVAL;
        } else if ((strcmp(argv[i], "-c")==0) || (strcmp(argv[i], "--window-count")==0)) {
            i++;
            if (i<argc) {
                window_count = strtol(argv[i], &endptr, base);
                if (errno != 0) {
                    perror("strtod");
                    errno = EXIT_FAILURE;
                }
                if (endptr == argv[i]) {
                    fprintf(stderr, "No digits were found\n");
                    errno = EXIT_FAILURE;
                }
            } else errno = EINVAL;
        } else if ((strcmp(argv[i], "-t")==0) || (strcmp(argv[i], "--f2-threshold")==0)) {
            i++;
            if (i<argc) {
                threshold_f2 = (long unsigned int)strtol(argv[i], &endptr, base);
                if (errno != 0) {
                    perror("strtod");
                    errno = EXIT_FAILURE;
                }
                if (endptr == argv[i]) {
                    fprintf(stderr, "No digits were found\n");
                    errno = EXIT_FAILURE;
                }
            } else errno = EINVAL;
        } else if ((strcmp(argv[i], "-h")==0) || (strcmp(argv[i], "--help")==0)) {
            print_help_message();
            return 0;
        } else if ((strcmp(argv[i], "-v")==0) || (strcmp(argv[i], "--verbose")==0)) {
            verbose = true;
        } else {
            errno = EINVAL;
        }
    }

    /* display param */
    if ((errno==0) && (verbose==true)) {
        printf("Arguments parsed:\n");
        printf("  hostname:     %s\n", hostname);
        printf("  interface:    %s\n", interface);
        printf("  window_size:  %ld\n", window_size);
        printf("  window_count: %ld\n", window_count);
        printf("  f2_threshold: %lu\n", threshold_f2);
        printf("  verbose:      %d\n", verbose);
        /* preventing interger overflow during printing */
        window_ws = 1 << window_size;
        window_wc = 1 << window_count;
        printf("Calculated value:\n");
        printf("  window duration (ns):     %ld\n", window_ws);
        printf("  windows in interval:      %ld\n", window_wc);
        printf("  interval duration (ns):   %ld\n", window_ws * window_wc);
    }

    /* creating measuring vector */
    if (errno==0) {
        //if (verbose) printf("creating measuring vector\n");
        packet_cnt_vec = (long int*)calloc((size_t)window_count, sizeof(long int));
        init_vec(packet_cnt_vec);
        if (verbose) print_vec(packet_cnt_vec); // used to check allocated vec status
    }

    /* creating packet structure */
    if (errno==0) {
        //if (verbose) printf("Creating packet structure\n");
        packet = trace_create_packet();
        if (packet == NULL) {
            perror("trace_create_packet");
            errno = LIBTRACE_ERRNO;
        }
        errno = 0; // resetting errno caused during trace opening process
    }
    
    /* creating input trace */
    if (errno==0) {
        //if (verbose) printf("Creating input trace\n");
        trace = trace_create(interface);
        if (trace_is_err(trace)) {
            trace_perror(trace, "trace_create");
            errno = LIBTRACE_ERRNO;
        }
        if (trace_start(trace) != 0) {
            trace_perror(trace, "trace_start");
            errno = LIBTRACE_ERRNO;
        }
        errno = 0; // resetting errno caused during trace opening process
    }

    /* prevent elapse_ns overflow */
    struct timespec init_ts;
    if (clock_gettime(CLOCK_REALTIME, &init_ts) == -1) {
        perror("clock_gettime");
    }
    if (errno == 0) {
        previous_s = init_ts.tv_sec;
        previous_ns = init_ts.tv_nsec;
    }
    /* main loop 
     *
     * Note that this is not a regular while loop that execute infinitely and exhaust CPU, this
     * loop wait for any packet coming through to trigger calculation
     *
     * when exiting, this loop will also wait till a packet to arrive to exit and cleanup resource
     */
    bool exit_loop = false;
    while ((trace_read_packet(trace, packet) > 0) && (exit_loop==false) && (exit_flag==false)) {
        if (filter_flag==false) {
            // if not filtering
            per_packet(packet);
        } else {
            // if filtering
            if (filter_applied==false) {
                // if filter already applied
                apply_filter();
            }
        }
        if (trace_is_err(trace)) {
            trace_perror(trace, "Reading packets");
            errno = LIBTRACE_ERRNO;
        }
        if (errno != 0) {
            if (errno == EAGAIN) ; // do nothing
            else exit_loop = true;
        }
    }

    if (verbose) printf("Cleaning up resource\n");
    /* these lines sometimes leads to unexpected behavior,
     * likely resource re-claim by OS
     */
    remove_filter();
    free(packet_cnt_vec);
    if (trace) trace_destroy(trace);
    if (packet) trace_destroy_packet(packet);
    if (errno == 0) {
        exit(EXIT_SUCCESS);
    } else {
        if (errno < LIBTRACE_ERRNO) printf("Exit with code %d: %s\n", errno, strerror(errno));
        else printf("Exit due to libtrace error\n");
        exit(errno);
    }
    return 0;
}
