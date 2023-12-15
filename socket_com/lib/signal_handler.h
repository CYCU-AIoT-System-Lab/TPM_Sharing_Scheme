/**
 * @file signal_handler.h
 * @brief Header file for signal_handler.c
 */

#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

#include "output_format.h"

/**
 * @brief Signal handler for SIGINT
 * @param signum Signal number
 * @param pFormat Output format
 */
sighandler_t SIGNAL_HANDLER_sigint_handler(int signum, output_format_t pFormat);

/**
 * @brief Signal handler for SIGTERM
 * @param signum Signal number
 * @param pFormat Output format
 */
sighandler_t SIGNAL_HANDLER_sigterm_handler(int signum, output_format_t pFormat);

/**
 * @brief Signal handler for SIGQUIT
 * @param signum Signal number
 * @param pFormat Output format
 */
sighandler_t SIGNAL_HANDLER_sigquit_handler(int signum, output_format_t pFormat);

/**
 * @brief Signal handler for SIGSEGV
 * @param signum Signal number
 * @param pFormat Output format
 */
sighandler_t SIGNAL_HANDLER_sigsegv_handler(int signum, output_format_t pFormat);

/**
 * @brief Multi signal handler register
 * @param pFormat Output format
 */
void SIGNAL_HANDLER_multi_register(output_format_t pFormat);

#endif // SIGNAL_HANDLER_H
