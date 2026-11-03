/**
 * @name:Matrix_Config.c
 * @brief:为Matrix_Hub提供硬件抽象层和相关配置
 * @changlog:2024-3-18 CYK-Dot 初次移植
*/

/* =============头文件========================================*/
#include "matrix_os_config.h"

/* ==============全局变量=====================================*/
RCS_MMU_T matrix_mmu;

/* ==============导出函数定义=================================*/

/**
 * @name:OS_Malloc
 * @brief:在此处填入实现内存分配的函数，以供Matrix_Hub使用
*/
void* OS_Malloc(uint16_t size_byte)
{
    #ifdef MALLOC_DISABLE
        return RCS_MMU_Malloc(&matrix_mmu,size_byte);
    #else
        return malloc(size_byte);
    #endif
}

/**
 * @name:OS_Free
 * @brief:在此处填入实现内存释放的函数，以供Matrix_Hub使用
*/
void OS_Free(void* mem_p)
{
    #ifdef MALLOC_DISABLE
        RCS_MMU_Free(&matrix_mmu,mem_p);
    #else
        free(mem_p);
    #endif
}

/**
 * @name:OS_Err_Collector
 * @brief:在此处实现收集日志的函数，以供Matrix_Hub展示相关信息
*/
void OS_Err_Collector(const char* format, ...)
{
    RCS_Shell_Logs(format);
}

/**
 * @name:OS_Commander
 * @brief:在此处实现终端控制的函数，以供Matrix_Hub控制终端显示
*/
void OS_Commander(const char* format, ...)
{
	
}

/**
 * @name:OS_Matrix_Init
 * @brief:如果上述几个功能需要初始化才能使用，则在此处实现初始化
*/
void OS_MatrixHub_Init(void)
{
    
}