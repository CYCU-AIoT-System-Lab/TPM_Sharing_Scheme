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
int main(int argc, char *argv[])
{
	output_format_t pFormat;
	init_output_format(&pFormat, "SERVER");
	printf("%sServer started!\n", pFormat.info);
	return 0;
}
