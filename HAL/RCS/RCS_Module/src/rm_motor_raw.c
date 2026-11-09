/**
 *@filename rm_motor_raw.c
 *@brief 大疆电机原始控制接口(直接封装CAN报文)
 *@author CYK-Dot@2025-05-20
 *--------------------------------------------------------------------------------------------------
**/

/* 头文件依赖 -------------------------------------------------------------------------------------*/
#include "rm_motor_raw.h"
#include "rcs_can.h"
#include "rcs_os.h"
#include <stdlib.h>

/* 私有宏定义 -------------------------------------------------------------------------------------*/

#define RM_FRONT          0x200  //ID1~4 电机
#define RM_BEHIND         0x1FF  //ID5~8 电机
#define RM_GM6020_V_FRONT 0x1FF  //ID1~4 (6020电压)
#define RM_GM6020_V_BEHIND 0x2FF //ID5~7 (6020电压)
#define RM_GM6020_FRONT   0x1FE  //ID1~4 (6020电流)
#define RM_GM6020_BEHIND  0x2FE  //ID5~8 (6020电流)

#define ENCODER2ANGLE     0.0439453125f  //360/8192
#define MAX_MOTOR_NUM     12 //RM3508(8个) + GM6020(4个)
#define MAX_CAN_NUM       2

/* 私有结构体定义 ---------------------------------------------------------------------------------*/

typedef struct {
    float start_angle;           // 起始角度
    uint16_t origin_angle;       // 原始角度值
    uint16_t last_angle;         // 上一次角度值
    int16_t cycles;             // 圈数计数
    volatile float float_angle;  // 浮点角度值
    volatile int32_t int_angle;  // 整数角度值
    int16_t origin_speed;        // 原始速度值
    volatile int16_t speed;      // 滤波后速度值
} motor_data_t;

/* 私有变量定义 -----------------------------------------------------------------------------------*/

// 下面三个变量用于维护rcs_can_t和电机数据的映射关系
// 因此内部顺序和CAN1、CAN2没有必然关系，只与初始化顺序有关系
// 例如先初始化了CAN2，再初始化CAN1，则g_can_handle[0]对应CAN2，g_motor_data[0]对应CAN2上的电机数据
motor_data_t g_motor_data[RCS_CAN_COUNT][MAX_MOTOR_NUM];
static rcs_can_t g_can_handle[RCS_CAN_COUNT];
static uint8_t g_can_index;

/* 私有函数声明 -----------------------------------------------------------------------------------*/

static void Int16_to_Int8(int16_t data, uint8_t *high, uint8_t *low);
static void RM_Motor_Data_Filter(rcs_can_t handle, const CAN_RxHeaderTypeDef* header, uint8_t* data);
static void RM_Motor_Encoder_Init(void);
static void RM_Motor_Angle_Init(void);
static uint8_t get_can_index(rcs_can_t handle);
static void process_motor_data(uint8_t can_idx, uint8_t motor_idx, uint16_t angle, int16_t speed);

/* 导出函数实现 -----------------------------------------------------------------------------------*/

/**
 * @brief 让CAN总线支持大疆电机
 * @param conf 如果CAN对象不存在时，本函数会创建一个，创建的CAN对象将使用本参数填入的配置
 * @param handle CAN对象
 * @return 是否成功
 * 
 * @note 不同于HAL库的CAN对象，RCS_CAN和CAN外设是一对一的关系。
 *       一条CAN总线只能有一个RCS_CAN对象与之对应，一个RCS_CAN对象也只能对应一条CAN总线。
 *       因此建议全局直接定死两个RCS_CAN对象指代两路CAN线，需要的时候再传给各类函数，不再额外定义新的RCS_CAN对象。
 * 
 * @warning 本函数必须在任务中调用，不要在ISR中调用，也不要在main中调用，因为vTaskDelay()函数需要RTOS任务上下文
 */
rcs_err_t rm_motor_init(CAN_HandleTypeDef *hcan,rcs_can_t RCS_CAN_x)
{
    rcs_kprintf("rm_motor:start init on CAN%d\n",portMAX_DELAY,RCS_CAN_x+1);
    // 初始化CAN
    rcs_err_t err = rcs_can_driver_install(hcan,RCS_CAN_x);
    if (err != RCS_OK) {
        rcs_kprintf("rm_motor:init failed,CAN install err.\n", portMAX_DELAY);
        return err;
    }
    // 如果此前CAN总线已经订阅过了大疆电机，则不再注册回调；否则注册回调并初始化电机状态变量
    if (rcs_can_is_subscribe(RCS_CAN_x, RM_Motor_Data_Filter) == RCS_OK) {
        rcs_kprintf("rm_motor:early OK,already subscribed before\n", portMAX_DELAY);
        return RCS_OK;
    }
    else {
        RM_Motor_Encoder_Init();
        if(rcs_can_subscribe(RCS_CAN_x, RM_Motor_Data_Filter) != RCS_OK) {
            rcs_kprintf("rm_motor:subscribe failed\n", portMAX_DELAY);
            return NULL;
        }
        vTaskDelay(100);
        RM_Motor_Angle_Init();
    }
	rcs_kprintf("rm_motor:init OK on CAN%d\n", portMAX_DELAY,RCS_CAN_x+1);
    // 成功
    return RCS_OK;
}

/**
 * @brief 发送电流给前4个大疆电机，支持RM3508和RM2006
 * @param RCS_CAN_x CAN编号
 * @param current1 电机1电流，单位mA
 * @param current2 电机2电流
 * @param current3 电机3电流
 * @param current4 电机4电流
 * @param timeout 超时时间，单位tick，可以使用pdMS_TO_TICKS(n)获取n毫秒对应的tick数
 * @return 返回错误码
 * 
 * @warning 对RM3508，最大电流-20000~20000mA，对RM2006，最大电流-10000~10000mA
 * @warning 本函数不应在中断中调用，因为vTaskDelay()函数需要RTOS任务上下文
 */
rcs_err_t rm_motor_send_current_rm3508_front(rcs_can_t RCS_CAN_x, int16_t current1, int16_t current2, int16_t current3, int16_t current4, TickType_t timeout)
{
    // 检查是否在中断中调用
    if (__get_IPSR() != 0) {
        return RCS_ERR_NOT_ALLOWED;
    }
    // 生成CAN报文
    CAN_TxHeaderTypeDef txMsg;
    uint8_t data[8];
    txMsg.StdId = RM_FRONT;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;
    Int16_to_Int8(current1, &data[0], &data[1]);
    Int16_to_Int8(current2, &data[2], &data[3]);
    Int16_to_Int8(current3, &data[4], &data[5]);
    Int16_to_Int8(current4, &data[6], &data[7]);
    // 发送CAN报文
    return rcs_can_send(RCS_CAN_x, &txMsg, data, timeout);
}

/**
 * @brief 发送电流给前4个GM6020电机 
 * @param RCS_CAN_x CAN编号
 * @param current1 电机1电流，单位mA
 * @param current2 电机2电流
 * @param current3 电机3电流
 * @param current4 电机4电流
 * @param timeout 超时时间，单位tick，可以使用pdMS_TO_TICKS(n)获取n毫秒对应的tick数
 */
rcs_err_t rm_motor_send_current_gm6020_front(rcs_can_t RCS_CAN_x, int16_t current1, int16_t current2, int16_t current3, int16_t current4, TickType_t timeout)
{
    // 检查是否在中断中调用
    if (__get_IPSR() != 0) {
        return RCS_ERR_NOT_ALLOWED;
    }

    CAN_TxHeaderTypeDef txMsg;
    uint8_t data[8];
    txMsg.StdId = RM_GM6020_FRONT;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    Int16_to_Int8(current1, &data[0], &data[1]);
    Int16_to_Int8(current2, &data[2], &data[3]);
    Int16_to_Int8(current3, &data[4], &data[5]);
    Int16_to_Int8(current4, &data[6], &data[7]);

    return rcs_can_send(RCS_CAN_x, &txMsg, data, 0);
}

/**
 * @brief 发送电压给前4个GM6020电机
 * @param RCS_CAN_x 哪一路CAN
 * @param voltage1 电机1电压，单位mV
 * @param voltage2 电机2电压
 * @param voltage3 电机3电压
 * @param voltage4 电机4电压    
 * @param timeout 超时时间，单位tick，可以使用pdMS_TO_TICKS(n)获取n毫秒对应的tick数
 * @return 返回错误码
 * 
 * @warning 本函数不应在中断中调用，因为vTaskDelay()函数需要RTOS任务上下文
 */
rcs_err_t rm_motor_send_voltage_gm6020_front(rcs_can_t RCS_CAN_x, int16_t voltage1, int16_t voltage2, int16_t voltage3, int16_t voltage4, TickType_t timeout)
{
    // 检查是否在中断中调用
    if (__get_IPSR() != 0) {
        return RCS_ERR_NOT_ALLOWED;
    }

    CAN_TxHeaderTypeDef txMsg;
    uint8_t data[8];
    txMsg.StdId = RM_GM6020_V_FRONT;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    Int16_to_Int8(voltage1, &data[0], &data[1]);
    Int16_to_Int8(voltage2, &data[2], &data[3]);
    Int16_to_Int8(voltage3, &data[4], &data[5]);
    Int16_to_Int8(voltage4, &data[6], &data[7]);

    return rcs_can_send(RCS_CAN_x, &txMsg, data, timeout);
}

/**
 * @brief 发送电流给后4个大疆电机，支持RM3508和RM2006
 * @param handle CAN对象
 * @param current5 电机5电流，单位mA
 * @param current6 电机6电流
 * @param current7 电机7电流
 * @param current8 电机8电流
 * @param timeout 超时时间，单位tick，可以使用pdMS_TO_TICKS(n)获取n毫秒对应的tick数
 * @return 返回错误码
 * 
 * @warning 对RM3508，最大电流-20000~20000mA，对RM2006，最大电流-10000~10000mA
 * @warning 本函数不应在中断中调用，因为vTaskDelay()函数需要RTOS任务上下文
 */
rcs_err_t rm_motor_send_current_rm3508_rear(rcs_can_t handle, int16_t current5, int16_t current6, int16_t current7, int16_t current8, TickType_t timeout)
{
    // 检查是否在中断中调用
    if (__get_IPSR() != 0) {
        return RCS_ERR_NOT_ALLOWED;
    }

    CAN_TxHeaderTypeDef txMsg;
    uint8_t data[8];
    txMsg.StdId = RM_BEHIND;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    Int16_to_Int8(current5, &data[0], &data[1]);
    Int16_to_Int8(current6, &data[2], &data[3]);
    Int16_to_Int8(current7, &data[4], &data[5]);
    Int16_to_Int8(current8, &data[6], &data[7]);

    return rcs_can_send(handle, &txMsg, data, timeout);
}

/**
 * @brief 发送电流给后4个GM6020电机
 * @param handle CAN对象
 * @param current5 电机5电流，单位mA
 * @param current6 电机6电流
 * @param current7 电机7电流
 * @param timeout 超时时间，单位tick，可以使用pdMS_TO_TICKS(n)获取n毫秒对应的tick数
 * @return 返回错误码
 * 
 * @warning 本函数不应在中断中调用，因为vTaskDelay()函数需要RTOS任务上下文
 */
rcs_err_t rm_motor_send_current_gm6020_rear(rcs_can_t handle, int16_t current5, int16_t current6, int16_t current7, TickType_t timeout)
{
    // 检查是否在中断中调用
    if (__get_IPSR() != 0) {
        return RCS_ERR_NOT_ALLOWED;
    }

    CAN_TxHeaderTypeDef txMsg;
    uint8_t data[8];
    txMsg.StdId = RM_GM6020_BEHIND;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    Int16_to_Int8(current5, &data[0], &data[1]);
    Int16_to_Int8(current6, &data[2], &data[3]);
    Int16_to_Int8(current7, &data[4], &data[5]);
    data[6] = 0;
    data[7] = 0;

    return rcs_can_send(handle, &txMsg, data, timeout);
}

/**
 * @brief 发送电压给后4个GM6020电机
 * @param handle CAN对象
 * @param voltage5 电机5电压，单位mV
 * @param voltage6 电机6电压
 * @param voltage7 电机7电压
 * @param timeout 超时时间，单位tick，可以使用pdMS_TO_TICKS(n)获取n毫秒对应的tick数
 * @return 返回错误码
 * 
 * @warning 本函数不应在中断中调用，因为vTaskDelay()函数需要RTOS任务上下文
 */
rcs_err_t rm_motor_send_voltage_gm6020_rear(rcs_can_t handle, int16_t voltage5, int16_t voltage6, int16_t voltage7, TickType_t timeout)
{
    // 检查是否在中断中调用
    if (__get_IPSR() != 0) {
        return RCS_ERR_NOT_ALLOWED;
    }

    CAN_TxHeaderTypeDef txMsg;
    uint8_t data[8];
    txMsg.StdId = RM_GM6020_V_BEHIND;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 8;

    Int16_to_Int8(voltage5, &data[0], &data[1]);
    Int16_to_Int8(voltage6, &data[2], &data[3]);
    Int16_to_Int8(voltage7, &data[4], &data[5]);
    data[6] = 0;
    data[7] = 0;

    return rcs_can_send(handle, &txMsg, data, timeout);
}

/**
 * @brief 获取电机角度，单位为度
 * @param handle CAN对象
 * @param id 电机ID
 * @return 返回电机角度，单位为度
 */
volatile uint8_t can_idxx;
float rm_motor_get_angle_rm3508_deg(rcs_can_t RCS_CAN_x,uint16_t id)
{
    return g_motor_data[RCS_CAN_x][id-1].float_angle;
}

/**
 * @brief 获取电机速度，单位为rpm
 * @param handle CAN对象
 * @param id 电机ID
 * @return 返回电机速度，单位为rpm
 */
float rm_motor_get_speed_rm3508_rpm(rcs_can_t RCS_CAN_x,uint16_t id)
{
    return g_motor_data[RCS_CAN_x][id-1].origin_speed;
}

/**
 * @brief 获取GM6020电机速度，单位为rpm
 * @param handle CAN对象
 * @param id 电机ID
 * @return 返回电机速度，单位为rpm
 */
float rm_motor_get_speed_gm6020_rpm(rcs_can_t RCS_CAN_x,uint16_t id)
{
    return g_motor_data[RCS_CAN_x][id+4].origin_speed;
}

/**
 * @brief 获取GM6020电机角度，单位为度
 * @param handle CAN对象
 * @param id 电机ID
 * @return 返回电机角度，单位为度
 */
float rm_motor_get_angle_gm6020_deg(rcs_can_t RCS_CAN_x,uint16_t id)
{
    return g_motor_data[RCS_CAN_x][id+4].float_angle;
}




/* 私有函数实现 -----------------------------------------------------------------------------------*/

static void Int16_to_Int8(int16_t data, uint8_t *high, uint8_t *low)
{
    *high = (data & 0xFF00) >> 8;
    *low = data & 0x00FF;
}

/**
 * @brief 大疆电机CAN总线订阅服务，解析大疆电机的转速和角度
 * 
 * @param handle 
 * @param header 
 * @param data 
 */
static void RM_Motor_Data_Filter(rcs_can_t RCS_CAN_x, const CAN_RxHeaderTypeDef* header, uint8_t* data)
{
    // 检查是否为标准数据帧且长度为8字节
    if ((header->IDE != CAN_ID_STD) || (header->RTR != CAN_RTR_DATA) || (header->DLC != 0x08)) {
        return;
    }
    // 解析电机编号
    uint8_t motor_idx = (header->StdId & 0x000f) - 1;
    if (motor_idx >= MAX_MOTOR_NUM) {
        return;
    }

    // 解析速度和角度数据
    uint16_t angle = (data[0] << 8) | data[1];
    int16_t speed = (data[2] << 8) | data[3];

    // 处理电机数据
    process_motor_data(RCS_CAN_x, motor_idx, angle, speed);
}

static void process_motor_data(uint8_t RCS_CAN_x, uint8_t motor_idx, uint16_t angle, int16_t speed)
{
    motor_data_t* motor = &g_motor_data[RCS_CAN_x][motor_idx];
    
    // 处理速度数据
    motor->origin_speed = speed;
    motor->speed = (abs(speed) <= 3) ? 0 : speed;

    // 处理角度数据
    motor->origin_angle = angle;
    
    // 跨0度线判定
    if (abs(angle - motor->last_angle) > 6000) {
        if (motor->last_angle > 4000) {
            motor->cycles++;
        } else {
            motor->cycles--;
        }
    }
    
    // 更新角度相关数据
    motor->last_angle = angle;
    motor->float_angle = 360.0f * motor->cycles + 
                        angle * ENCODER2ANGLE - 
                        motor->start_angle;
    motor->int_angle = (int32_t)motor->float_angle;
}

static void RM_Motor_Encoder_Init(void)
{
    for (int i = 0; i < MAX_CAN_NUM; i++) {
        for (int j = 0; j < MAX_MOTOR_NUM; j++) {
            motor_data_t* motor = &g_motor_data[i][j];
            motor->start_angle = 0;
            motor->origin_angle = 0;
            motor->cycles = 0;
            motor->float_angle = 0;
            motor->int_angle = 0;
            motor->origin_speed = 0;
            motor->speed = 0;
        }
    }
}

static void RM_Motor_Angle_Init(void)
{
    for (int i = 0; i < MAX_CAN_NUM; i++) {
        for (int j = 0; j < MAX_MOTOR_NUM; j++) {
            g_motor_data[i][j].start_angle = g_motor_data[i][j].float_angle;
        }
    }
}
