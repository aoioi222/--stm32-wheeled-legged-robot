#include <math.h>

#include "easy_filters.h"

/**
 * @brief 低通滤波器对象私有成员
 * 
 */
typedef struct
{
    float alpha;
    float output;
}LowpassHandle_t;

/**
 * @brief 创建一个一阶低通滤波器
 * 
 * @param alpha 
 * @return LowpassFilter_t 
 */
LowpassFilter_t xLowpassFilterCreate(float alpha)
{
    if (alpha < 0.0f || alpha > 1.0f || isnan(alpha)) {
        return NULL;
    }

    LowpassHandle_t *handle = (LowpassHandle_t *)FILTER_PORT_MALLOC(sizeof(LowpassHandle_t));
    if (handle == NULL) 
        return NULL;

    handle->output = NAN;
    handle->alpha = alpha;
    return (LowpassFilter_t)handle;
}

/**
 * @brief 异步向低通滤波器中送入数据
 * 
 * @param filter 
 * @param dataToFilter 
 * @return int 
 */
int xLowpassFilterSend(LowpassFilter_t filter,float dataToFilter)
{
    if (filter == NULL) 
        return FILTER_ERR_INVALID_PARAM;
    LowpassHandle_t *handle = (LowpassHandle_t*)filter;

    if (isnan(handle->output)) {
        handle->output = dataToFilter;
        return FILTER_NOT_FINISH;
    }
    handle->output = handle->output * handle->alpha + dataToFilter * (1.0f - handle->alpha);
    return FILTER_OK;
}

/**
 * @brief 异步从低通滤波器中读取数据
 * 
 * @param filter 
 * @return float 
 */
float xLowpassFilterRecv(LowpassFilter_t filter)
{
    if (filter == NULL) 
        return NAN;
    LowpassHandle_t *handle = (LowpassHandle_t*)filter;
    
    if (isnan(handle->output)) {
        return NAN;
    }
    return handle->output;
}

/**
 * @brief 窗口滤波器对象私有成员
 * 
 */
typedef struct 
{
    float *buffer;       
    int size;            
    int head;            
    int count;           
    float sum;        
} WindowFilterHandle_t;

/**
 * @brief 实例化一个窗口均值滤波器对象
 * 
 * @param windowSize 
 * @return WindowFilter_t 
 */
WindowFilter_t xWindowFilterCreate(uint16_t windowSize)
{
    WindowFilterHandle_t *handle = (WindowFilterHandle_t *)FILTER_PORT_MALLOC(sizeof(WindowFilterHandle_t));
    if (!handle) 
        return NULL;
    if (windowSize == 0) 
        return NULL;

    handle->buffer = (float *)FILTER_PORT_MALLOC(windowSize * sizeof(float));
    if (!handle->buffer) {
        FILTER_PORT_FREE(handle);
        return NULL;
    }
    handle->size = windowSize;
    handle->head = 0;
    handle->count = 0;
    handle->sum = 0.0f;
    return handle;
}

/**
 * @brief 异步向窗口滤波器中添加数据
 * 
 * @param filter 
 * @param dataToFilter 
 * @return int 
 */
int xWindowFilterSend(WindowFilter_t filter, float dataToFilter)
{
    WindowFilterHandle_t *handle = (WindowFilterHandle_t *)filter;
    if (!handle || isnan(dataToFilter) || isinf(dataToFilter)) 
        return FILTER_ERR_INVALID_PARAM;

    // 窗口未满
    if (handle->count < handle->size) {
        handle->buffer[handle->head] = dataToFilter;
        handle->sum += dataToFilter;
        handle->head = (handle->head + 1) % handle->size;
        handle->count++;
    }
    // 窗口已满 
    else {
        float old = handle->buffer[handle->head];
        handle->sum -= old;
        handle->buffer[handle->head] = dataToFilter;
        handle->sum += dataToFilter;
        handle->head = (handle->head + 1) % handle->size;
    }
    return FILTER_OK;
}

/**
 * @brief 异步从窗口滤波器中取出数据
 * 
 * @param filter 
 * @return float 
 */
float xWindowFilterRecv(WindowFilter_t filter)
{
    WindowFilterHandle_t *handle = (WindowFilterHandle_t *)filter;
    if (!handle || handle->count == 0) 
        return NAN;
    return handle->sum / handle->count;
}

/**
 * @brief 卡尔曼滤波器对象私有成员
 * 
 */
typedef struct 
{
    float estimate;  
    float P;     
    float Q;      
    float R;      
    float K;      
}KalmanFilterHandle_t;

/**
 * @brief 实例化一个一维卡尔曼滤波器对象
 * 
 * @param Q 
 * @param R 
 * @param initialEstimate 
 * @return KalmanFilter_t 
 */
KalmanFilter_t xKalmanFilterCreate(float Q, float R, float initialEstimate) 
{
    KalmanFilterHandle_t* handle = (KalmanFilterHandle_t*)FILTER_PORT_MALLOC(sizeof(KalmanFilterHandle_t));
    if (!handle) 
        return NULL;
    if (Q < 0.0f || R < 0.0f)
        return NULL;

    handle->estimate = initialEstimate;
    handle->P = 1.0f;
    handle->Q = Q;
    handle->R = R;
    handle->K = 0.0f;
    return (KalmanFilter_t)handle;
}

/**
 * @brief 异步向卡尔曼滤波器对象添加数据
 * 
 * @param filter 
 * @param measuredValue 
 * @return int 
 */
int xKalmanFilterSend(KalmanFilter_t filter, float measuredValue) 
{
    if (!filter) 
        return FILTER_ERR_INVALID_PARAM;
    if (isnan(measuredValue))
        return FILTER_ERR_INVALID_PARAM;
    KalmanFilterHandle_t* handle = (KalmanFilterHandle_t*)filter;

    handle->P += handle->Q;
    handle->K = handle->P / (handle->P + handle->R);
    handle->estimate += handle->K * (measuredValue - handle->estimate);
    handle->P *= (1.0f - handle->K);
    return FILTER_OK;
}

/**
 * @brief 异步从卡尔曼滤波器对象读取数据
 * 
 * @param filter 
 * @return float 
 */
float xKalmanFilterRecv(KalmanFilter_t filter) 
{
    if (!filter) 
        return NAN;
    KalmanFilterHandle_t* handle = (KalmanFilterHandle_t*)filter;
    return handle->estimate;
}