#include <stdio.h>
#include <stdlib.h>
#include "output_format.h"
#include "lib_system.h"

void exit_program(int exit_code, output_format_t pFormat) {
	if (exit_code == 0)
		printf("%sExiting program with code %d\n", pFormat.success, exit_code);
	else if (exit_code == 1)
		printf("%sExiting program with code %d\n", pFormat.error, exit_code);
	else
		printf("%sExiting program with code %d\n", pFormat.warning, exit_code);
	printf("%sServer stopped!\n", pFormat.info);
	free_output_format(&pFormat);
	exit(exit_code);
}
