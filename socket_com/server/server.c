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
	printf("Hello from server!\n");
	output_format_t *pFormat = init_output_format("SERVER");
	printf("%sHello from server!\n", pFormat->info);;
	return 0;
}
