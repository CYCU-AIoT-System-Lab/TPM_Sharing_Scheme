#ifndef LIB_SYSTEM_H
#define LIB_SYSTEM_H

#include "output_format.h"

/**
 * @brief Exit program with exit code and free output format
 * @param exit_code Exit code
 * @param pFormat Output format
 * @return void
 */
void exit_program(int exit_code, output_format_t pFormat);

#endif // LIB_SYSTEM_H
