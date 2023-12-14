/**
 * @file output_format.c
 * @brief Output format settings
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "output_format.h"

/// Maximum number of characters in a format buffer string
#define char_buffer_size 256

void init_output_format(output_format_t *pFormat, char *msg)
{
	char buf[char_buffer_size];

	// formats
	pFormat->reset = "\033[0m";
	pFormat->bold = "\033[1m";
	pFormat->red_foreground = "\033[31m";
	pFormat->green_foreground = "\033[32m";
	pFormat->yellow_foreground = "\033[33m";
	pFormat->blue_foreground = "\033[34m";
	// integrated formats
	snprintf(buf,
			sizeof(buf),
			"%s%s%s%s%s%s",
			pFormat->bold,
			pFormat->blue_foreground,
			"[",
			msg,
			"] ",
			pFormat->reset);
	pFormat->info = strdup(buf);
	snprintf(buf,
			sizeof(buf),
			"%s%s%s%s%s%s",
			pFormat->bold,
			pFormat->green_foreground,
			"[",
			msg,
			"-SUCCESS] ",
			pFormat->reset);
	pFormat->success = strdup(buf);
	snprintf(buf,
			sizeof(buf),
			"%s%s%s%s%s%s",
			pFormat->bold,
			pFormat->yellow_foreground,
			"[",
			msg,
			"-WARN] ",
			pFormat->reset);
	pFormat->warning = strdup(buf);
	snprintf(buf,
			sizeof(buf),
			"%s%s%s%s%s%s",
			pFormat->bold,
			pFormat->red_foreground,
			"[",
			msg,
			"-ERROR] ",
			pFormat->reset);
	pFormat->error = strdup(buf);
	// actions
	pFormat->reset_terminal = "\033c";
	pFormat->clear_screen_below = "\033[0J";
	pFormat->move_cursor_up = "\033[1A";
	pFormat->delete_line = "\033[0M";

	return;
}

void free_output_format(output_format_t *pFormat) {
	free(pFormat->info);
	free(pFormat->success);
	free(pFormat->warning);
	free(pFormat->error);
	return;
}
