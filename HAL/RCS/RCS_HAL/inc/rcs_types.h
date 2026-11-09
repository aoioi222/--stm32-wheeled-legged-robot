#pragma once


typedef int rcs_err_t;

#define RCS_OK 0                    ///< 没有问题
#define RCS_FAIL -1                 ///< 未知原因导致失败
#define RCS_ERR_INVALID_PARAM -2    ///< 函数的参数填错了
#define RCS_ERR_OVERFLOW      -3    ///< 超过最大限制
#define RCS_ERR_MULTIPLE_USED -4    ///< 同一个东西反复使用
#define RCS_ERR_HW            -5    ///< 硬件内部出错
#define RCS_ERR_BUSY          -6    ///< 设备正忙
#define RCS_ERR_NO_MEM        -7    ///< 内存不足
#define RCS_ERR_NOT_ALLOWED   -8    ///< 不允许的操作

#define PI        3.141592653589793238463
#define DEG2RAD   1.74532925199433e-2f
#define RAD2DEG   5.72957795130823e1f

#define ZERO      1e-10f
#define NEAR_ZERO 1e-3f
