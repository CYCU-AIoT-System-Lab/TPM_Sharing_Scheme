/**
 * @file server.c
 * @brief Server program for the TPM project
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
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
	int sfd, cfd; // server and client file descriptors
	struct sockaddr_in saddr, caddr; // server and client addresses
	// Main process --> server init
	sfd = socket(AF_INET, SOCK_STREAM, 0); // IPv4, TCP, default protocol
	if (sfd == -1) {
		printf("%sError opening socket!\n", pFormat.error);
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSocket opened!\n", pFormat.success);
	}
	memset(&saddr, 0, sizeof(saddr)); // clear structure
	saddr.sin_family = AF_INET; // IPv4
	saddr.sin_port = htons(80); // port 80 (TCP)
	saddr.sin_addr.s_addr = htonl(0x7F000001); // localhost 127.0.0.1
	if (bind(sfd, (struct sockaddr *) &saddr, sizeof(saddr)) == -1) {
		printf("%sError binding socket!\n", pFormat.error);
		printf("%s%s\n", pFormat.error, strerror(errno));
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSocket binded!\n", pFormat.success);
	}
	if (listen(sfd, 10) == -1) { // 10 is the maximum number of pending connections
		printf("%sError listening socket!\n", pFormat.error);
		printf("%s%s\n", pFormat.error, strerror(errno));
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSocket listening!\n", pFormat.success);
	}
	// Main process --> client connection
	while (1) {
		cfd = accept(sfd, (struct sockaddr *) &caddr, sizeof(caddr));
		if (cfd == -1) {
			printf("%sError accepting client!\n", pFormat.error);
			printf("%s%s\n", pFormat.error, strerror(errno));
			LIB_SYSTEM_exit_program(1, pFormat);
		} else {
			printf("%sClient accepted!\n", pFormat.success);
		}
		// Main process --> client communication
		fclose(cfd);
	}
	// End
	close(sfd);
	LIB_SYSTEM_exit_program(0, pFormat);
	return 0;
}
