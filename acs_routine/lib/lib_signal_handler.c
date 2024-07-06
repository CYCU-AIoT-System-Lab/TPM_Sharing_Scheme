#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>

#include "lib_signal_handler.h"

/* https://stackoverflow.com/questions/43667012
 */
void signal_handler (int signum) {
    char *signal_name = strsignal(signum);
    extern int exit_flag;
    printf("\nSignal %2d -> %s received.\n", signum, signal_name);
    if (signum != SIGSEGV) {
        printf("Wait till main loop finishes\n");
        exit_flag = true;
        errno = EINTR;
    } else exit(SIGSEGV);
}

/* custom signal to apply filter
 *
 * no remove filter signal implemented due to
 * that any hostname required to be filtered
 * externally should not be allow back in,
 * plus, the main program will clear the filter
 * once it terminates
 */
void signal_USR1_handler (int signum) {
    char *signal_name = strsignal(signum);
    extern int filter_flag;
    if (filter_flag==false) {
        printf("\nSignal %2d -> %s received, aplying filter.\n", signum, signal_name);
        filter_flag = true;
    } else {
        printf("Signal %2d -> %s received, filter already applied.\n", signum, signal_name);
    }
}

void register_all_signal_handlers (void) {
    signal(SIGABRT, signal_handler);
    signal(SIGFPE, signal_handler);
    signal(SIGILL, signal_handler);
    signal(SIGINT, signal_handler);
    signal(SIGPIPE, signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGSEGV, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGUSR1, signal_USR1_handler);
}
