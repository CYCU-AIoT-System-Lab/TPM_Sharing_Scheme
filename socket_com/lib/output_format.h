/**
 * @file output_format.h
 * @brief Header file for otuput_format.c
 * Ref:
 * 1. https://github.com/belongtothenight/PRML_Code/blob/main/src/libc/output_format.h
 */

#ifndef OUTPUT_FORMAT_H
#define OUTPUT_FORMAT_H

/**
 * @brief Output format struct for "socket_com" project
 * @param reset reset all formats
 * @param bold bold format
 * @param red_foreground red foreground format
 * @param green_foreground green foreground format
 * @param yellow_foreground yellow foreground format
 * @param info info message
 * @param success success message
 * @param warning warning message
 * @param error error message
 * @param reset_terminal reset terminal
 * @param clear_screen_below clear screen below
 * @param move_cursor_up move cursor up
 * @param delete_line delete line
 */
typedef struct{
	// formats
	char *reset;
	char *bold;
	char *red_foreground;
	char *green_foreground;
	char *yellow_foreground;
	char *blue_foreground;
	// integrated formats
	char *info;
	char *success;
	char *warning;
	char *error;
	// actions
	char *reset_terminal;
	char *clear_screen_below;
	char *move_cursor_up;
	char *delete_line;
} output_format_t;

/**
 * @brief Initialize output format
 * @param pFormat output format struct
 * @param msg Message to print between brackets
 * @return void
 */
void init_output_format(output_format_t *pFormat, char *msg);

#endif // OUTPUT_FORMAT_H
