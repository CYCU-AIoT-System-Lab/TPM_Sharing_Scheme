/**
 * @file signal_handler.h
 * @brief Header file for signal_handler.c
 */

#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

#include <signal.h>
#include "output_format.h"

/**
 * @brief Signal handler
 * @param signum Signal number
 */
void SIGNAL_HANDLER_signal_handler(int signum);

/**
 * @brief Signal handler register
 * @param pFormat Output format
 */
void SIGNAL_HANDLER_register(output_format_t pFormat);



#endif // SIGNAL_HANDLER_H
