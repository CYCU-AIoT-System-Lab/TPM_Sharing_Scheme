/**
 * @file server.c
 * @brief Server program for the TPM project
 */
#include <stdio.h>
#include <stdlib.h>

/**
 * @brief Main function for the server part of project
 * @param argc Number of arguments
 * @param argv Array of arguments
 * @return 0 on success, 1 on failure
 */
int main(int argc, char *argv[])
{
	printf("Hello from server!\n");
	char *heap = malloc(10);
	heap[10] = 1;
	return 0;
}
