/**
 * @file server.c
 * @brief Server program for the TPM project
 */

#include <stdio.h>
#include <stdlib.h>
#include "../lib/output_format.h"

/**
 * @brief Main function for the server part of project
 * @param argc Number of arguments
 * @param argv Array of arguments
 * @return 0 on success, 1 on failure
 */
int main(int argc, char *argv[]) {
	// Start up
	output_format_t pFormat;
	init_output_format(&pFormat, "SERVER");
	printf("%sServer started!\n", pFormat.info);
	// Input Arguments
	if (argc != 2) {
		printf("%sUsage: %s <config file>\n", pFormat.warning, argv[0]);
		exit_program(1, pFormat);
	}
	// Main process
	// End
	exit_program(0, pFormat);
	return 0;
}

/**
 * @brief Exit program with exit code and free output format
 * @param exit_code Exit code
 * @param pFormat Output format
 * @return void
 */
void exit_program(int exit_code, output_format_t pFormat) {
	printf("Exiting program with code %d\n", exit_code);
	printf("%sServer stopped!\n", pFormat.info);
	free_output_format(&pFormat);
	exit(exit_code);
}
