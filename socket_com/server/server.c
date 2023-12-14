/**
 * @file server.c
 * @brief Server program for the TPM project
 */

#include <stdio.h>
#include <stdlib.h>
#include "../lib/output_format.h"
#include "../lib/lib_system.h"

/**
 * @brief Main function for the server part of project
 * @param argc Number of arguments
 * @param argv Array of arguments
 * @return 0 on success, 1 on failure
 */
int main(int argc, char *argv[]) {
	// Start up
	output_format_t pFormat;
	OUTPUT_FORMAT_init_output_format(&pFormat, "SERVER");
	printf("%sServer started!\n", pFormat.info);
	// Input Arguments
	if (argc != 2) {
		printf("%sInvalid number of arguments: %d!\n", pFormat.error, argc);
		printf("%sUsage: %s <config file>\n", pFormat.error, argv[0]);
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sConfig file: %s\n", pFormat.success, argv[1]);
	}
	// Main process
	// End
	LIB_SYSTEM_exit_program(0, pFormat);
	return 0;
}
