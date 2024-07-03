/* @file main
 * @brief Main function for ACS attestation log DB anomaly detection,
 *        No error handling is implemented in this version.
 * @authoer belongtothenight / Da-Chuan Chen/ 2024
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>

void print_help_message (void);

int main (int argc, char *argv[]) {
    /* params */
                        errno = 0;          /* error number */
    int                 i;                  /* iterator */
    char               *endptr;             /* string conversion */
    const char         *host_name = NULL;   /* MySQL host name */
    const char         *user_name = NULL;   /* MySQL user name */
    const char         *password = NULL;    /* MySQL password */
    const char         *db_name = NULL;     /* MySQL database name */
    const char         *port_number = NULL; /* MySQL port number */
    double              time_interval = 0;  /* time interval */
    struct timespec     start_time;         /* start time */
    struct timespec     end_time;           /* start time */
    time_t              elapsed_time_sec;   /* elapsed time in seconds */

    /* CLI parsing */
    for (i=1; (i<argc) && (errno==0); i++) {
        /* argument pairs <flag> <value> */
        if ((strcmp(argv[i], "-H") == 0) || (strcmp(argv[i], "--mysql-host") == 0)) {
            i++;
            if (i < argc) {
                host_name = argv[i];
            } else {
                errno = EINVAL;
            }
        } else if ((strcmp(argv[i], "-u") == 0) || (strcmp(argv[i], "--mysql-user") == 0)) {
            i++;
            if (i < argc) {
                user_name = argv[i];
            } else {
                errno = EINVAL;
            }
        } else if ((strcmp(argv[i], "-p") == 0) || (strcmp(argv[i], "--mysql-password") == 0)) {
            i++;
            if (i < argc) {
                password = argv[i];
            } else {
                errno = EINVAL;
            }
        } else if ((strcmp(argv[i], "-d") == 0) || (strcmp(argv[i], "--mysql-db") == 0)) {
            i++;
            if (i < argc) {
                db_name = argv[i];
            } else {
                errno = EINVAL;
            }
        } else if ((strcmp(argv[i], "-P") == 0) || (strcmp(argv[i], "--mysql-port") == 0)) {
            i++;
            if (i < argc) {
                port_number = argv[i];
            } else {
                errno = EINVAL;
            }
        } else if ((strcmp(argv[i], "-i") == 0) || (strcmp(argv[i], "--check-interval") == 0)) {
            i++;
            if (i < argc) {
                time_interval = strtod(argv[i], &endptr);
                if (errno != 0) {
                    perror("strtod");
                    exit(EXIT_FAILURE);
                }
                if (endptr == argv[i]) {
                    fprintf(stderr, "No digits were found\n");
                    exit(EXIT_FAILURE);
                }
                if (*endptr != 0) {
                    printf("Further characters after number: \"%s\"\n", endptr);
                    errno = EINVAL;
                }
            } else {
                errno = EINVAL;
            }
        } else if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "--help") == 0)) {
            print_help_message();
            exit(EXIT_SUCCESS);
        } else {
            errno = EINVAL;
        }
    }
    if (errno == 0) {
        printf("Arguments parsed:\n");
        printf("  MySQL host:       %s\n", host_name);
        printf("  MySQL user name:  %s\n", user_name);
        printf("  MySQL password:   %s\n", password);
        printf("  MySQL database:   %s\n", db_name);
        printf("  MySQL port:       %s\n", port_number);
        printf("  Check interval:   %f\n", time_interval);
    }
    exit(EXIT_SUCCESS);
}

void print_help_message (void) {
    printf("Usage: ./attestlog_AD -H <host> -u <user_name> -p <password> -d <db_name> -P <port> -i <sec>\n");
    printf("Usage: ./attestlog_AD -h");
    printf("Options:\n");
    printf("  -H, --mysql-host <host_name>      MySQL host name, IPv4 address\n");
    printf("  -u, --mysql-user <user_name>      MySQL user name\n");
    printf("  -p, --mysql-password <password>   MySQL password\n");
    printf("  -d, --mysql-db <db_name>          MySQL database name\n");
    printf("  -P, --mysql-port <port_number>    MySQL port number\n");
    printf("  -i, --check-interval <sec>        Check interval in seconds\n");
    printf("  -h, --help                        Show this help message and exit\n");
    return;
}
