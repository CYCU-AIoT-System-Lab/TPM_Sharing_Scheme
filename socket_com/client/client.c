/**
 * @file client.c
 * @brief Client program for the TPM project
 */

#include <stdio.h>
#include <stdlib.h>
#include "../lib/output_format.h"
#include "../lib/lib_system.h"

/**
 * @brief Main function for the client part of project
 * @param argc Number of arguments
 * @param argv Array of arguments
 * @return 0 on success, 1 on failure
 */
int main(int argc, char *argv[]) {
	// Start up
	output_format_t pFormat;
	OUTPUT_FORMAT_init_output_format(&pFormat, "CLIENT");
	printf("%sClient started!\n", pFormat.info);
	// Main process
	// End
	LIB_SYSTEM_exit_program(0, pFormat);
	return 0;
}
