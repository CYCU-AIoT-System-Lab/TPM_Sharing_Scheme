/**
 * @file signal_handler.c
 * @brief Library for signal handling
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "output_format.h"
#include "lib_system.h"
#include "signal_handler.h"

sighandler_t SIGNAL_HANDLER_sigint_handler(int signum, output_format_t pFormat) {
	printf("%sReceived signal %d: %s\n", pFormat.warning, signum, "SIGINT");
	LIB_SYSTEM_exit_program(0, pFormat);
}

sighandler_t SIGNAL_HANDLER_sigterm_handler(int signum, output_format_t pFormat) {
	printf("%sReceived signal %d: %s\n", pFormat.warning, signum, "SIGTERM");
	LIB_SYSTEM_exit_program(0, pFormat);
}

sighandler_t SIGNAL_HANDLER_sigquit_handler(int signum, output_format_t pFormat) {
	printf("%sReceived signal %d: %s\n", pFormat.warning, signum, "SIGQUIT");
	LIB_SYSTEM_exit_program(0, pFormat);
}

sighandler_t SIGNAL_HANDLER_sigsegv_handler(int signum, output_format_t pFormat) {
	printf("%sReceived signal %d: %s\n", pFormat.warning, signum, "SIGSEGV");
	LIB_SYSTEM_exit_program(0, pFormat);
}

void SIGNAL_HANDLER_multi_register(output_format_t pFormat) {
	if (signal(SIGINT, SIGNAL_HANDLER_sigint_handler) == SIG_ERR) {
		printf("%sError registering SIGINT handler!\n", pFormat.error);
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSIGINT handler registered!\n", pFormat.success);
	}
	if (signal(SIGTERM, SIGNAL_HANDLER_sigterm_handler) == SIG_ERR) {
		printf("%sError registering SIGTERM handler!\n", pFormat.error);
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSIGTERM handler registered!\n", pFormat.success);
	}
	if (signal(SIGQUIT, SIGNAL_HANDLER_sigquit_handler) == SIG_ERR) {
		printf("%sError registering SIGQUIT handler!\n", pFormat.error);
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSIGQUIT handler registered!\n", pFormat.success);
	}
	if (signal(SIGSEGV, SIGNAL_HANDLER_sigsegv_handler) == SIG_ERR) {
		printf("%sError registering SIGSEGV handler!\n", pFormat.error);
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSIGSEGV handler registered!\n", pFormat.success);
	}
}
