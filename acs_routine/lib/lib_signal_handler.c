#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>

#include "lib_signal_handler.h"

extern int exit_flag;

/* https://stackoverflow.com/questions/43667012
 */
void signal_handler (int signum) {
    char *signal_name = strsignal(signum);
    printf("\nSignal %2d -> %s received.\n", signum, signal_name);
    if (signum != SIGSEGV) {
        printf("Wait till main loop finishes\n");
        exit_flag = true;
        errno = EINTR;
    } else exit(SIGSEGV);
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
}
