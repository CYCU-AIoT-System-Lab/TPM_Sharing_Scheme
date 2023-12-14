/**
 * @file client.c
 * @brief Client program for the TPM project
 */

#include <stdio.h>
#include <stdlib.h>
#include "../lib/output_format.h"

/**
 * @brief Main function for the client part of project
 * @param argc Number of arguments
 * @param argv Array of arguments
 * @return 0 on success, 1 on failure
 */
int main(int argc, char *argv[]) {
	// Start up
	output_format_t pFormat;
	init_output_format(&pFormat, "CLIENT");
	printf("%sClient started!\n", pFormat.info);
	// Main process
	// End
	printf("%sClient stopped!\n", pFormat.info);
	free_output_format(&pFormat);
	return 0;
}
