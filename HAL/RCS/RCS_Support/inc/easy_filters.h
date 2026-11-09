#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

// 系统调用接口
#define FILTER_PORT_MALLOC            malloc
#define FILTER_PORT_FREE              free

// 错误码
#define FILTER_OK 0
#define FILTER_NOT_FINISH -1
#define FILTER_ERR_INVALID_PARAM -2

// 一阶低通滤波器类
typedef void* LowpassFilter_t;
LowpassFilter_t xLowpassFilterCreate(float alpha);
int xLowpassFilterSend(LowpassFilter_t filter,float dataToFilter);
float xLowpassFilterRecv(LowpassFilter_t filter);

// 均值窗口滤波器类
typedef void* WindowFilter_t;
WindowFilter_t xWindowFilterCreate(uint16_t windowSize);
int xWindowFilterSend(WindowFilter_t filter,float dataToFilter);
float xWindowFilterRecv(WindowFilter_t filter);

// 一阶卡尔曼滤波器类
typedef void* KalmanFilter_t;
KalmanFilter_t xKalmanFilterCreate(float Q,float R,float initEstimate);
int xKalmanFilterSend(KalmanFilter_t filter, float measuredValue) ;
float xKalmanFilterRecv(KalmanFilter_t filter) ;


