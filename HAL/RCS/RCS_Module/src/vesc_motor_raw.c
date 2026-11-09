/**
 * @filename vesc_motor_raw.c
 * @brief VESC电调原始控制接口(直接封装CAN报文)
 * @author CYK-Dot@2025-05-31
 *
 * @note 目前本文件提供的功能并未完全涵盖VESC支持的CAN报文。
 *       作为开源项目，本杰明的文档并不齐全，不像RM3508那样有明确的报文说明，需要通过阅读源码得知报文结构。
 *       可以上github下载VESC源代码(https://github.com/vedderb)，阅读后对本文件进行扩展。
 *--------------------------------------------------------------------------------------------------
**/

/* 头文件引入 --------------------------------------------------------------------------------*/

#include "vesc_motor_raw.h"
#include "rcs_can.h"

/* 私有宏定义 --------------------------------------------------------------------------------*/
typedef enum {
	VESC_PACKET_SET_DUTY = 0,
	VESC_PACKET_SET_CURRENT,
	VESC_PACKET_SET_CURRENT_BRAKE,
	VESC_PACKET_SET_RPM,
	VESC_PACKET_SET_POS,
	VESC_PACKET_FILL_RX_BUFFER,
	VESC_PACKET_FILL_RX_BUFFER_LONG,
	VESC_PACKET_PROCESS_RX_BUFFER,
	VESC_PACKET_PROCESS_SHORT_BUFFER,
	VESC_PACKET_STATUS,
	VESC_PACKET_SET_CURRENT_REL,          //Set current relative to the minimum and maximum current limits.
	VESC_PACKET_SET_CURRENT_BRAKE_REL,
	VESC_PACKET_SET_CURRENT_HANDBRAKE,
	VESC_PACKET_SET_CURRENT_HANDBRAKE_REL,
	VESC_PACKET_STATUS_2,
	VESC_PACKET_STATUS_3,
	VESC_PACKET_STATUS_4,
	VESC_PACKET_PING,
	VESC_PACKET_PONG,
	VESC_PACKET_DETECT_APPLY_ALL_FOC,
	VESC_PACKET_DETECT_APPLY_ALL_FOC_RES,
	VESC_PACKET_CONF_CURRENT_LIMITS,
	VESC_PACKET_CONF_STORE_CURRENT_LIMITS,
	VESC_PACKET_CONF_CURRENT_LIMITS_IN,
	VESC_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	VESC_PACKET_CONF_FOC_ERPMS,
	VESC_PACKET_CONF_STORE_FOC_ERPMS,
	VESC_PACKET_STATUS_5,
	VESC_PACKET_POLL_TS5700N8501_STATUS,
	VESC_PACKET_CONF_BATTERY_CUT,
	VESC_PACKET_CONF_STORE_BATTERY_CUT,
	VESC_PACKET_SHUTDOWN,
	VESC_PACKET_IO_BOARD_ADC_1_TO_4,
	VESC_PACKET_IO_BOARD_ADC_5_TO_8,
	VESC_PACKET_IO_BOARD_ADC_9_TO_12,
	VESC_PACKET_IO_BOARD_DIGITAL_IN,
	VESC_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL,
	VESC_PACKET_IO_BOARD_SET_OUTPUT_PWM,
	VESC_PACKET_BMS_V_TOT,
	VESC_PACKET_BMS_I,
	VESC_PACKET_BMS_AH_WH,
	VESC_PACKET_BMS_V_CELL,
	VESC_PACKET_BMS_BAL,
	VESC_PACKET_BMS_TEMPS,
	VESC_PACKET_BMS_HUM,
	VESC_PACKET_BMS_SOC_SOH_TEMP_STAT,
	VESC_PACKET_PSW_STAT,
	VESC_PACKET_PSW_SWITCH,
	VESC_PACKET_BMS_HW_DATA_1,
	VESC_PACKET_BMS_HW_DATA_2,
	VESC_PACKET_BMS_HW_DATA_3,
	VESC_PACKET_BMS_HW_DATA_4,
	VESC_PACKET_BMS_HW_DATA_5,
	VESC_PACKET_BMS_AH_WH_CHG_TOTAL,
	VESC_PACKET_BMS_AH_WH_DIS_TOTAL,
	VESC_PACKET_UPDATE_PID_POS_OFFSET,
	VESC_PACKET_POLL_ROTOR_POS,
	VESC_PACKET_BMS_BOOT,
	VESC_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
} VESC_PACKET_ID;

/* 静态函数声明 ------------------------------------------------------------------------------*/

static inline void Int32_to_Int8(int32_t Input,uint8_t* Output);

/* 导出函数定义 ------------------------------------------------------------------------------*/

/**
 * @brief 让CAN总线支持VESC电调
 * 
 * @param conf CAN总线配置
 * @param handle CAN总线句柄
 * @return rcs_err_t 是否成功
 */
rcs_err_t vesc_motor_init(const rcs_can_conf_t *conf,rcs_can_t *handle)
{
    if (*handle == NULL) {
        *handle = rcs_can_create(conf);
    }
    return RCS_OK;
}

/**
 * @brief 发送电流控制命令到VESC电调
 * 
 * @param handle CAN总线句柄
 * @param id VESC电调ID(可在VESC上位机配置)
 * @param current 电流，单位mA
 * @param timeout 超时时间戳，可以通过pdMS_TO_TICKS()获取
 * @return rcs_err_t 是否成功
 * 
 * @warning 本函数不应在中断上下文或main函数上下文中被调用
 */
rcs_err_t vesc_motor_send_current(rcs_can_t handle,uint16_t id,int32_t current,TickType_t timeout)
{
    // 检查是否在中断中调用
    if (__get_IPSR() != 0) {
        return RCS_ERR_NOT_ALLOWED;
    }
    // 生成CAN报文中的数据
    volatile uint8_t vesc_cmd_data[4];
	Int32_to_Int8(current,vesc_cmd_data);
    // 生成CAN报文中的帧头
    CAN_TxHeaderTypeDef txMsg;
    txMsg.ExtId = (VESC_PACKET_SET_CURRENT<<8) | ext_id;
    txMsg.IDE = CAN_ID_EXT;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 4;
    // 发送CAN报文
    return rcs_can_send(handle, &txMsg, vesc_cmd_data, timeout);
}

/**
 * @brief 发送转速控制命令到VESC电调
 * 
 * @param handle CAN总线句柄
 * @param id VESC电调ID(可在VESC上位机配置)
 * @param erpm 转速，单位为磁极转速，比如这个电机有16对磁极，那么真实的转速为erpm/16
 * @param timeout 超时时间戳，可以通过pdMS_TO_TICKS()获取
 * @return rcs_err_t 是否成功
 * 
 * @note VESC电调的pid要在上位机里调整，这部分不由主控负责，和3508不一样
 * @warning 本函数不应在中断上下文或main函数上下文中被调用
 */
rcs_err_t vesc_motor_send_rpm(rcs_can_t handle,uint16_t id,int32_t erpm,TickType_t timeout)
{
    // 检查是否在中断中调用
    if (__get_IPSR() != 0) {
        return RCS_ERR_NOT_ALLOWED;
    }
    // 生成CAN报文中的数据
    volatile uint8_t vesc_cmd_data[4];
	Int32_to_Int8(erpm,vesc_cmd_data);
    // 生成CAN报文中的帧头
    CAN_TxHeaderTypeDef txMsg;
    txMsg.ExtId = (VESC_PACKET_SET_RPM<<8) | ext_id;
    txMsg.IDE = CAN_ID_EXT;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 4;
    // 发送CAN报文
    return rcs_can_send(handle, &txMsg, vesc_cmd_data, timeout);
}

/**
 * @brief 发送角度控制命令到VESC电调
 * 
 * @param handle CAN总线句柄
 * @param id VESC电调ID(可在VESC上位机配置)
 * @param pos 角度，单位度
 * @param timeout 超时时间戳，可以通过pdMS_TO_TICKS()获取
 * @return rcs_err_t 是否成功
 * 
 * @note VESC电调的pid要在上位机里调整，这部分不由主控负责，和3508不一样
 * @warning 本函数不应在中断上下文或main函数上下文中被调用
 */
rcs_err_t vesc_motor_send_pos(rcs_can_t handle,uint16_t id,float pos,TickType_t timeout)
{

}

/* 静态函数定义 ------------------------------------------------------------------------------*/

static inline void Int32_to_Int8(int32_t Input,uint8_t* Output)
{
	Output[3]= Input & 0xff;
	Output[2]=(Input & 0xff00) >> 8;
	Output[1]=(Input & 0xff0000) >> 16;
	Output[0]=(Input & 0xff000000) >> 24;
}