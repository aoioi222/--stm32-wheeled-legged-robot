/* ========.h head=====================================*/
#ifndef MATRIX_OS_SUPPORT_H_
#define MATRIX_OS_SUPPORT_H_

/* ========includes=====================================*/
#include "stdint.h"

#include "matrix_config.h"

#include "RCS_RTOS.h"

/* ========interface funtion===============================*/
void* OS_Malloc(uint16_t size_byte);
void OS_Free(void* mem_p);
void OS_Err_Collector(const char* format, ...);
void OS_Commander(const char* format, ...);
void OS_MatrixHub_Init(void);
#endif