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
#include <time.h>
#include <mysql/mysql.h>
#include <stdbool.h>

void print_help_message (void) {
    printf("Usage:\n");
    printf("  ./attestlog_AD -H <host> -u <user_name> -p <password> -d <db_name> -P <port> [-v]\n");
    printf("  ./attestlog_AD -h\n");
    printf("Options:\n");
    printf("  -H, --mysql-host <host_name>      MySQL host name, IPv4 address\n");
    printf("  -u, --mysql-user <user_name>      MySQL user name\n");
    printf("  -p, --mysql-password <password>   MySQL password\n");
    printf("  -d, --mysql-db <db_name>          MySQL database name\n");
    printf("  -P, --mysql-port <port_number>    MySQL port number\n");
    printf("  -v, --verbose                     Verbose output\n");
    printf("  -h, --help                        Show this help message and exit\n");
    printf("Output format:\n");
    printf("  <result_num>:<id>:<userid>:<hostname>:<result_info>\n");
    printf("Description:\n");
    printf("  This program will query the last/latest row from the attestlog table in provided MySQL database and determine attestation result.\n");
    return;
}

MYSQL_ROW query_last_line (MYSQL *con, const char* query_str) {
    MYSQL_RES *res;
    if (mysql_query(con, query_str)) {
        fprintf(stderr, "Failed to query from database: Error: %s\nQuery:%s", mysql_error(con), query_str);
    }
    res=mysql_use_result(con);
    return mysql_fetch_row(res);
}

struct ACS_state {
    bool is_yes;
    bool is_no;
    bool is_null;
    const char *p;
};

struct ACS_query {
    const char *id;
    const char *userid;
    const char *hostname;
    struct ACS_state quoteverified;
    struct ACS_state pcrschanged;
    struct ACS_state pcrinvalid;
};

/* this function will trigger aggregate value warning */
struct ACS_state mysql_strtol (struct ACS_state acs_state) {
    long val;
    int base = 10;
    char *endptr;
    const char *item = acs_state.p;
    errno = 0;
    /* printf here can leads to seg fault */
    //printf("%s\n", item);
    if (acs_state.p != NULL) {
        val = strtol(item, &endptr, base);
        if (errno != 0) {
            perror("strtol");
            exit(EXIT_FAILURE);
        }
        if (endptr == acs_state.p) {
            fprintf(stderr, "No digits were found\n");
            exit(EXIT_FAILURE);
        }
        if (val == 1) acs_state.is_yes = true;
        else if (val == 0) acs_state.is_no = true;
        else printf("unexpected value");
    } else acs_state.is_null = true;
    return acs_state;
}

int main (int argc, char *argv[]) {
    /* params */
                        errno = 0;                  /* error number */
    int                 i;                          /* iterator */
    int                 base = 10;                  /* 10 base strtol conversion */
    char               *endptr;                     /* string conversion */
    const char         *host_name = NULL;           /* MySQL host name */
    const char         *user_name = NULL;           /* MySQL user name */
    const char         *password = NULL;            /* MySQL password */
    const char         *db_name = NULL;             /* MySQL database name */
    unsigned int        port_number = 0;            /* MySQL port number */
    bool                verbose = false;            /* verbose output */
    MYSQL               mysql;                      /* MySQL connection */
    MYSQL_ROW           mysql_row;                  /* MySQL queried row */
    struct ACS_query    selected_data = {0};        /* Selected queried data */

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
                port_number = (unsigned int)strtol(argv[i], &endptr, base);
                if (errno != 0) {
                    perror("strtol");
                    exit(EXIT_FAILURE);
                }
                if (endptr == argv[i]) {
                    fprintf(stderr, "No digits were found\n");
                    exit(EXIT_FAILURE);
                }
                if (*endptr != '\0') {
                    printf("Further characters after number: \"%s\"\n", endptr);
                    errno = EINVAL;
                }
            } else {
                errno = EINVAL;
            }
        } else if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "--help") == 0)) {
            print_help_message();
            exit(EXIT_SUCCESS);
        } else if ((strcmp(argv[i], "-v") == 0) || (strcmp(argv[i], "--verbose") == 0)) {
            verbose = true;
        } else {
            errno = EINVAL;
        }
    }
    /* display CLI arguments */
    if ((errno == 0) && (verbose == true)) {
        printf("Arguments parsed:\n");
        printf("    MySQL host:       %s\n", host_name);
        printf("    MySQL user name:  %s\n", user_name);
        printf("    MySQL password:   %s\n", password);
        printf("    MySQL database:   %s\n", db_name);
        printf("    MySQL port:       %d\n", port_number);
        printf("    Verbose:          %d\n", verbose);
    }

    /* connect to DB */
    if (errno == 0) {
        if (verbose) printf("Connecting to DB\n");
        mysql_init(&mysql);
        while (1) {
            if (!mysql_real_connect(&mysql,
                                    host_name,
                                    user_name,
                                    password,
                                    db_name,
                                    port_number,
                                    NULL,
                                    0)) {
                fprintf(stderr, "Failed to connect to database: Error: %s\n", mysql_error(&mysql));
                if (errno == EAGAIN) {
                    sleep(1);
                    printf("retry to connect to databse");
                }
            }
            //printf("Established DB connection\n");
            errno = 0; // resetting errno caused during DB connection
            break;
        }
    }

    /* get multiple info from attestlog last row */
    if (errno == 0) {
        if (verbose) printf("Quering data\n");
        /* MAX(id)      -> last row
         * MAX(id-1)    -> last-1 row
         * MIN(id)      -> first row
         * */
        mysql_row=query_last_line(&mysql, "SELECT id, userid, hostname, quoteverified, pcrschanged, pcrinvalid FROM attestlog WHERE id=(SELECT MAX(id) FROM attestlog);");
        if (mysql_row != NULL) {
            selected_data.id = mysql_row[0];
            selected_data.userid = mysql_row[1];
            selected_data.hostname = mysql_row[2];
            selected_data.quoteverified.p = mysql_row[3];
            selected_data.pcrschanged.p = mysql_row[4];
            selected_data.pcrinvalid.p = mysql_row[5];

            if (verbose) {
                printf("Queried data:\n");
                printf("    0>>%s\n", selected_data.id);
                printf("    1>>%s\n", selected_data.userid);
                printf("    2>>%s\n", selected_data.hostname);
                printf("    3>>%s\n", selected_data.quoteverified.p);
                printf("    4>>%s\n", selected_data.pcrschanged.p);
                printf("    5>>%s\n", selected_data.pcrinvalid.p);
            }

            //printf("Required data queried\n");
            errno = 0; // resetting errno caused during DB connection
        }
    }

    /* MySQL tinyint type value checking */
    if (errno == 0) {
        if (verbose) printf("Value checking\n");
        /* these 3 lines will trigger aggregate value warning */
        selected_data.quoteverified = mysql_strtol(selected_data.quoteverified);
        selected_data.pcrschanged = mysql_strtol(selected_data.pcrschanged);
        selected_data.pcrinvalid= mysql_strtol(selected_data.pcrinvalid);
        errno = 0; // resetting errno caused during DB connection
    }

    /* Case decision
     * | quoteverified | pcrschanged | pcrinvalid | case                |
     * + --------------+-------------+------------+---------------------+
     * | 1             | 0           | NULL       | enroll              |
     * | 1             | 0           | 0          | attestation success |
     * | else                                     | fail                |
     * */
    if (errno == 0) {
        if ((selected_data.quoteverified.is_yes) && (selected_data.pcrschanged.is_no) && (selected_data.pcrinvalid.is_null)) {
            printf("0:%s:%s:%s:Client enroll\n", selected_data.id, selected_data.userid, selected_data.hostname);
        } else if ((selected_data.quoteverified.is_yes) && (selected_data.pcrschanged.is_no) && (selected_data.pcrinvalid.is_no)) {
            printf("1:%s:%s:%s:Attestation succeed\n", selected_data.id, selected_data.userid, selected_data.hostname);
        } else {
            printf("2:%s:%s:%s:Attestation failed\n", selected_data.id, selected_data.userid, selected_data.hostname);
        }
    }

    if (errno == 0) {
        mysql_close(&mysql);
        exit(EXIT_SUCCESS);
    } else {
        printf("Exist with code %d: %s\n", errno, strerror(errno));
        exit(errno);
    }
    return 0;
}
