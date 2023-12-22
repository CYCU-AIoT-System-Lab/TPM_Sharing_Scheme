/**
 * @file server.c
 * @brief Server program for the TPM project
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "../lib/output_format.h"
#include "../lib/lib_system.h"

#define MAX_BUFFER_SIZE 1024

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
	printf("%sServer started\n", pFormat.info);
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
	int s_pton; // status
	struct sockaddr_in saddr, caddr; // server and client addresses
	socklen_t caddr_len = sizeof(caddr);
	char buffer[MAX_BUFFER_SIZE];
	uint16_t port = 80;
	char *ipv4_addr_str = "127.0.0.1";
	// Main process --> server init
	memset(&saddr, 0, sizeof(saddr)); // clear structure
	saddr.sin_family = AF_INET; // IPv4
	saddr.sin_port = htons(port); // port 80 (TCP)
	s_pton = inet_pton(AF_INET, ipv4_addr_str, &saddr.sin_addr.s_addr);
	if (s_pton <= 0) {
		if (s_pton == 0) {
			printf("%sNote in presentation format: %s\n", pFormat.error, ipv4_addr_str);
		} else {
			printf("%sError converting address: %s\n", pFormat.error, ipv4_addr_str);
			printf("%s%s\n", pFormat.error, strerror(errno));
		}
	}
	sfd = socket(AF_INET, SOCK_STREAM, 0); // IPv4, TCP, default protocol
	if (sfd == -1) {
		printf("%sError opening socket!\n", pFormat.error);
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSocket opened\n", pFormat.success);
	}
	if (bind(sfd, (struct sockaddr *) &saddr, sizeof(saddr)) == -1) {
		printf("%sError binding socket!\n", pFormat.error);
		printf("%s%s\n", pFormat.error, strerror(errno));
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSocket binded to %s:%lu\n", 
				pFormat.success, 
				//(unsigned long)ipv4_addr, 
				ipv4_addr_str,
				(unsigned long)port);
	}
	if (listen(sfd, 10) == -1) { // 10 is the maximum number of pending connections
		printf("%sError listening socket!\n", pFormat.error);
		printf("%s%s\n", pFormat.error, strerror(errno));
		LIB_SYSTEM_exit_program(1, pFormat);
	} else {
		printf("%sSocket listening\n", pFormat.success);
	}
	// Main process --> client connection
	while (1) {
		cfd = accept(sfd, (struct sockaddr *) &caddr, &caddr_len);
		if (inet_ntop(AF_INET, &caddr, ipv4_addr_str, INET_ADDRSTRLEN) == NULL) {
			printf("%sError converting address: %s\n", pFormat.error, ipv4_addr_str);
			printf("%s%s\n", pFormat.error, strerror(errno));
			LIB_SYSTEM_exit_program(1, pFormat);
		} else {
			printf("%sClient connected from %s:%lu\n", 
					pFormat.success, 
					ipv4_addr_str, 
					(unsigned long)ntohs(caddr.sin_port));
		}
		if (cfd == -1) {
			printf("%sError accepting client!\n", pFormat.error);
			printf("%s%s\n", pFormat.error, strerror(errno));
			LIB_SYSTEM_exit_program(1, pFormat);
		} else {
			printf("%sClient accepted from 0x%lx:0d%lu\n", 
					pFormat.success, 
					(unsigned long)ntohl(caddr.sin_addr.s_addr), 
					(unsigned long)ntohs(caddr.sin_port));
		}
		// Main process --> client communication
		ssize_t nread = recv(cfd, buffer, MAX_BUFFER_SIZE - 1, 0);
		if (nread > 0) {
			printf("%sReceived %ld bytes from client\n", pFormat.success, nread);
			buffer[nread] = '\0'; // null-terminate string
			printf("Received MSG: %s\n", buffer);
		} else {
			printf("%sError receiving data from client!\n", pFormat.error);
			printf("%s%s\n", pFormat.error, strerror(errno));
		}
		// Main process --> client disconnection
		if (close(cfd) == -1) {
			printf("%sError closing client!\n", pFormat.error);
			printf("%s%s\n", pFormat.error, strerror(errno));
			LIB_SYSTEM_exit_program(1, pFormat);
		} else {
			printf("%sClient closed\n", pFormat.success);
		}
	}
	// End
	LIB_SYSTEM_exit_program(0, pFormat);
	return 0;
}
