/**
  ******************************************************************************
  * @file    wit_c_sdk.c
  * @author  Wit
  * @version V1.0
  * @date    05-May-2022
  * @brief   This file provides all Configure sensor function.
  ******************************************************************************
  * @attention
  *
  *        http://wit-motion.cn/
  *
  ******************************************************************************
  *
  * @author CYK-Dot
  * @version V1.1.0
  * @date    2025-05-23
  * @brief
  * 1. 将维特智能提供的SDK进行面向对象化，允许同时存在和使用多个IMU实例
  * 2. 保留维特智能提供的系统调用接口，并在rcs_wit_gyro.cpp中部署到RCSLIB中
  * 3. 重新组织维特智能提供的.h文件，符合RCS阅读习惯
  * 
  ******************************************************************************
  */
#include "wit_c_sdk.h"
#define FuncW 0x06
#define FuncR 0x03

static const uint8_t __auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};
static const uint8_t __auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};


static uint16_t __CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    int i = 0;
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    for (; i<usDataLen; i++)
    {
    	uIndex = uchCRCHi ^ puchMsg[i];
    	uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
    	uchCRCLo = __auchCRCLo[uIndex];
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}


static uint8_t __CaliSum(uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t ucCheck = 0;
    for(i=0; i<len; i++) ucCheck += *(data + i);
    return ucCheck;
}


/**
 * @brief 向句柄中注册串口写入函数
 * @param handle 句柄
 * @param Write_func 写入函数
 * @return 返回值
 */
int32_t WitSerialWriteRegister(wit_gyro_t handle, SerialWrite Write_func)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(!Write_func)return WIT_HAL_INVAL;
    p_handle->p_WitSerialWriteFunc = Write_func;
    return WIT_HAL_OK;
}


static void CopeWitData(wit_gyro_t handle, uint8_t ucIndex, uint16_t *p_data, uint32_t uiLen)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 0, uiReg2Len = 0;
    uint16_t *p_usReg1Val = p_data;
    uint16_t *p_usReg2Val = p_data+3;
    
    uiReg1Len = 4;
    switch(ucIndex)
    {
        case WIT_ACC:   uiReg1 = AX;    uiReg1Len = 3;  uiReg2 = TEMP;  uiReg2Len = 1;  break;
        case WIT_ANGLE: uiReg1 = Roll;  uiReg1Len = 3;  uiReg2 = VERSION;  uiReg2Len = 1;  break;
        case WIT_TIME:  uiReg1 = YYMM;	break;
        case WIT_GYRO:  uiReg1 = GX;  uiLen = 3;break;
        case WIT_MAGNETIC: uiReg1 = HX;  uiLen = 3;break;
        case WIT_DPORT: uiReg1 = D0Status;  break;
        case WIT_PRESS: uiReg1 = PressureL;  break;
        case WIT_GPS:   uiReg1 = LonL;  break;
        case WIT_VELOCITY: uiReg1 = GPSHeight;  break;
        case WIT_QUATER:    uiReg1 = q0;  break;
        case WIT_GSA:   uiReg1 = SVNUM;  break;
        case WIT_REGVALUE:  uiReg1 = p_handle->uiReadRegIndex;  break;
		default:
			return ;
    }
    if(uiLen == 3)
    {
        uiReg1Len = 3;
        uiReg2Len = 0;
    }
    if(uiReg1Len)
	  {
        memcpy(&p_handle->sReg[uiReg1], p_usReg1Val, uiReg1Len<<1);
        p_handle->p_WitRegUpdateCbFunc(handle,uiReg1, uiReg1Len);
	  }
    if(uiReg2Len)
	  {
        memcpy(&p_handle->sReg[uiReg2], p_usReg2Val, uiReg2Len<<1);
        p_handle->p_WitRegUpdateCbFunc(handle,uiReg2, uiReg2Len);
	  }
}

/**
 * @brief 向句柄中更新串口收到的数据
 * @param handle 句柄
 * @param ucData 数据
 */
void WitSerialDataIn(wit_gyro_t handle, uint8_t ucData)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    uint16_t usCRC16, usTemp, i, usData[4];
    uint8_t ucSum;

    if(p_handle->p_WitRegUpdateCbFunc == NULL)return ;
    p_handle->ucWitDataBuff[p_handle->uiWitDataCnt++] = ucData;
    switch(p_handle->uiProtoclo)
    {
        case WIT_PROTOCOL_JY61:
        case WIT_PROTOCOL_NORMAL:
            if(p_handle->ucWitDataBuff[0] != 0x55)
            {
                p_handle->uiWitDataCnt--;
                memcpy(p_handle->ucWitDataBuff, &p_handle->ucWitDataBuff[1], p_handle->uiWitDataCnt);
                return ;
            }
            if(p_handle->uiWitDataCnt >= 11)
            {
                ucSum = __CaliSum(p_handle->ucWitDataBuff, 10);
                if(ucSum != p_handle->ucWitDataBuff[10])
                {
                    p_handle->uiWitDataCnt--;
                    memcpy(p_handle->ucWitDataBuff, &p_handle->ucWitDataBuff[1], p_handle->uiWitDataCnt);
                    return ;
                }
                usData[0] = ((uint16_t)p_handle->ucWitDataBuff[3] << 8) | (uint16_t)p_handle->ucWitDataBuff[2];
                usData[1] = ((uint16_t)p_handle->ucWitDataBuff[5] << 8) | (uint16_t)p_handle->ucWitDataBuff[4];
                usData[2] = ((uint16_t)p_handle->ucWitDataBuff[7] << 8) | (uint16_t)p_handle->ucWitDataBuff[6];
                usData[3] = ((uint16_t)p_handle->ucWitDataBuff[9] << 8) | (uint16_t)p_handle->ucWitDataBuff[8];
                CopeWitData(handle, p_handle->ucWitDataBuff[1], usData, 4);
                p_handle->uiWitDataCnt = 0;
            }
        break;
        case WIT_PROTOCOL_905x_MODBUS:
        case WIT_PROTOCOL_MODBUS:
            if(p_handle->uiWitDataCnt > 2)
            {
                if(p_handle->ucWitDataBuff[1] != FuncR)
                {
                    p_handle->uiWitDataCnt--;
                    memcpy(p_handle->ucWitDataBuff, &p_handle->ucWitDataBuff[1], p_handle->uiWitDataCnt);
                    return ;
                }
                if(p_handle->uiWitDataCnt < (p_handle->ucWitDataBuff[2] + 5))return ;
                usTemp = ((uint16_t)p_handle->ucWitDataBuff[p_handle->uiWitDataCnt-2] << 8) | p_handle->ucWitDataBuff[p_handle->uiWitDataCnt-1];
                usCRC16 = __CRC16(p_handle->ucWitDataBuff, p_handle->uiWitDataCnt-2);
                if(usTemp != usCRC16)
                {
                    p_handle->uiWitDataCnt--;
                    memcpy(p_handle->ucWitDataBuff, &p_handle->ucWitDataBuff[1], p_handle->uiWitDataCnt);
                    return ;
                }
                usTemp = p_handle->ucWitDataBuff[2] >> 1;
                for(i = 0; i < usTemp; i++)
                {
                    p_handle->sReg[i+p_handle->uiReadRegIndex] = ((uint16_t)p_handle->ucWitDataBuff[(i<<1)+3] << 8) | p_handle->ucWitDataBuff[(i<<1)+4];
                }
                p_handle->p_WitRegUpdateCbFunc(handle,p_handle->uiReadRegIndex, usTemp);
                p_handle->uiWitDataCnt = 0;
            }
        break;
		case WIT_PROTOCOL_905x_CAN:
        case WIT_PROTOCOL_CAN:
        case WIT_PROTOCOL_I2C:
        p_handle->uiWitDataCnt = 0;
        break;
    }
    if(p_handle->uiWitDataCnt == WIT_DATA_BUFF_SIZE)p_handle->uiWitDataCnt = 0;
}


/**
 * @brief 向句柄中注册I2C写入函数
 * @param handle 句柄
 * @param write_func 写入函数
 * @param read_func 读取函数
 * @return 返回值
 */
int32_t WitI2cFuncRegister(wit_gyro_t handle, WitI2cWrite write_func, WitI2cRead read_func)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(!write_func)return WIT_HAL_INVAL;
    if(!read_func)return WIT_HAL_INVAL;
    p_handle->p_WitI2cWriteFunc = write_func;
    p_handle->p_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}


/**
 * @brief 向句柄中注册CAN写入函数
 * @param handle 句柄
 * @param Write_func 写入函数
 * @return 返回值
 */
int32_t WitCanWriteRegister(wit_gyro_t handle, CanWrite Write_func)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(!Write_func)return WIT_HAL_INVAL;
    p_handle->p_WitCanWriteFunc = Write_func;
    return WIT_HAL_OK;
}

/**
 * @brief 向句柄中更新CAN收到的数据
 * @param handle 句柄
 * @param ucData 数据
 * @param ucLen 长度
 */
void WitCanDataIn(wit_gyro_t handle, uint8_t ucData[8], uint8_t ucLen)
{
	wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
	uint16_t usData[3];
	uint8_t  ucTemp; 
    
	if(p_handle->p_WitRegUpdateCbFunc == NULL)return ;
	if(ucLen < 8)return ;
	switch(p_handle->uiProtoclo)
	  {            
		  case WIT_PROTOCOL_905x_CAN:
			if(ucData[0] != 0x55) return;	
			if(ucData[1] == 0x53)
				{
			       switch(ucData[2])
					{
					    case 0X01: ucTemp = LRoll; break;
					    case 0X02: ucTemp = LPitch; break;
					    case 0X03: ucTemp = LYaw; break;
					}
                    p_handle->sReg[ucTemp]   = ((uint16_t)ucData[5] << 8) | ucData[4];    	
					p_handle->sReg[ucTemp+1] = ((uint16_t)ucData[7] << 8) | ucData[6];	
			        p_handle->p_WitRegUpdateCbFunc(handle,ucTemp, 2);
					break;
			    }	
		  case WIT_PROTOCOL_CAN:
            if(ucData[0] != 0x55) return;
            usData[0] = ((uint16_t)ucData[3] << 8) | ucData[2];
            usData[1] = ((uint16_t)ucData[5] << 8) | ucData[4];
            usData[2] = ((uint16_t)ucData[7] << 8) | ucData[6];
            CopeWitData(handle, ucData[1], usData, 3);
            break;
				
          case WIT_PROTOCOL_NORMAL:
          case WIT_PROTOCOL_905x_MODBUS:
          case WIT_PROTOCOL_MODBUS:
          case WIT_PROTOCOL_I2C:
          break;
      }
}

/**
 * @brief 向句柄中注册回调函数
 * @param handle 句柄
 * @param update_func 回调函数
 * @return 返回值
 */
int32_t WitRegisterCallBack(wit_gyro_t handle, RegUpdateCb update_func)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(!update_func)return WIT_HAL_INVAL;
    p_handle->p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}

/**
 * @brief 向句柄中写入寄存器
 * @param handle 句柄
 * @param uiReg 寄存器
 * @param usData 数据
 * @return 返回值
 */
int32_t WitWriteReg(wit_gyro_t handle, uint32_t uiReg, uint16_t usData)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    uint16_t usCRC;
    uint8_t ucBuff[8];
    if(uiReg >= REGSIZE)return WIT_HAL_INVAL;
    switch(p_handle->uiProtoclo)
    {
        case WIT_PROTOCOL_JY61:
            return WIT_HAL_INVAL;
        case WIT_PROTOCOL_NORMAL:
            if(p_handle->p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_handle->p_WitSerialWriteFunc(handle,ucBuff, 5);
            break;
        case WIT_PROTOCOL_905x_MODBUS:
        case WIT_PROTOCOL_MODBUS:
            if(p_handle->p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = p_handle->ucAddr;
            ucBuff[1] = FuncW;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = usData >> 8;
            ucBuff[5] = usData & 0xff;
            usCRC = __CRC16(ucBuff, 6);
            ucBuff[6] = usCRC >> 8;
            ucBuff[7] = usCRC & 0xff;
            p_handle->p_WitSerialWriteFunc(handle,ucBuff, 8);
            break;
		case WIT_PROTOCOL_905x_CAN:
        case WIT_PROTOCOL_CAN:
            if(p_handle->p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xff;
            ucBuff[4] = usData >> 8;
            p_handle->p_WitCanWriteFunc(handle,p_handle->ucAddr, ucBuff, 5);
            break;
        case WIT_PROTOCOL_I2C:
            if(p_handle->p_WitI2cWriteFunc == NULL)return WIT_HAL_EMPTY;
            ucBuff[0] = usData & 0xff;
            ucBuff[1] = usData >> 8;
			if(p_handle->p_WitI2cWriteFunc(handle,p_handle->ucAddr << 1, uiReg, ucBuff, 2) != 1)
			{
				 return  WIT_HAL_ERROR;
			} 
      break;
	default: 
            return WIT_HAL_INVAL;        
    }
    return WIT_HAL_OK;
}

/**
 * @brief 读取寄存器
 * @param handle 句柄
 * @param uiReg 寄存器
 * @param uiReadNum 读取数量
 * @return 返回值
 */
int32_t WitReadReg(wit_gyro_t handle, uint32_t uiReg, uint32_t uiReadNum)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    uint16_t usTemp, i;
    uint8_t ucBuff[8];
    if((uiReg + uiReadNum) >= REGSIZE)return WIT_HAL_INVAL;
    switch(p_handle->uiProtoclo)
    {
		case WIT_PROTOCOL_JY61: 
			  return WIT_HAL_INVAL;
        case WIT_PROTOCOL_NORMAL:
			  if(uiReadNum > 4)return WIT_HAL_INVAL;
              if(p_handle->p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
              ucBuff[0] = 0xFF;
              ucBuff[1] = 0xAA;
              ucBuff[2] = 0x27;
              ucBuff[3] = uiReg & 0xff;
              ucBuff[4] = uiReg >> 8;
              p_handle->p_WitSerialWriteFunc(handle,ucBuff, 5);
		   break;
        case WIT_PROTOCOL_905x_MODBUS:
        case WIT_PROTOCOL_MODBUS:
			  if(p_handle->p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
              usTemp = uiReadNum << 1;
              if((usTemp + 5) > WIT_DATA_BUFF_SIZE)return WIT_HAL_NOMEM;
              ucBuff[0] = p_handle->ucAddr;
              ucBuff[1] = FuncR;
              ucBuff[2] = uiReg >> 8;
              ucBuff[3] = uiReg & 0xFF;
              ucBuff[4] = uiReadNum >> 8;
              ucBuff[5] = uiReadNum & 0xff;
              usTemp = __CRC16(ucBuff, 6);
              ucBuff[6] = usTemp >> 8;
              ucBuff[7] = usTemp & 0xff;
              p_handle->p_WitSerialWriteFunc(handle,ucBuff, 8);
		   break;
	    case WIT_PROTOCOL_905x_CAN:
        case WIT_PROTOCOL_CAN:
			  if(uiReadNum > 3)return WIT_HAL_INVAL;
              if(p_handle->p_WitCanWriteFunc == NULL)return WIT_HAL_EMPTY;
              ucBuff[0] = 0xFF;
              ucBuff[1] = 0xAA;
              ucBuff[2] = 0x27;
              ucBuff[3] = uiReg & 0xff;
              ucBuff[4] = uiReg >> 8;
              p_handle->p_WitCanWriteFunc(handle,p_handle->ucAddr, ucBuff, 5);
           break;
        case WIT_PROTOCOL_I2C:
		      if(p_handle->p_WitI2cReadFunc == NULL)return WIT_HAL_EMPTY;
              usTemp = uiReadNum << 1;
              if(WIT_DATA_BUFF_SIZE < usTemp)return WIT_HAL_NOMEM;
              if(p_handle->p_WitI2cReadFunc(handle,p_handle->ucAddr << 1, uiReg, p_handle->ucWitDataBuff, usTemp) == 1)
              {
                  if(p_handle->p_WitRegUpdateCbFunc == NULL)return WIT_HAL_EMPTY;
                  for(i = 0; i < uiReadNum; i++)
                  {
                    p_handle->sReg[i+uiReg] = ((uint16_t)p_handle->ucWitDataBuff[(i<<1)+1] << 8) | p_handle->ucWitDataBuff[i<<1];
                  }
                  p_handle->p_WitRegUpdateCbFunc(handle,uiReg, uiReadNum);
              }
			break;
		default: 
                 return WIT_HAL_INVAL;
    }
    p_handle->uiReadRegIndex = uiReg;

    return WIT_HAL_OK;
}

/**
 * @brief 生成一个IMU对象，或是继承一个IMU对象
 * @param handle 句柄
 * @param uiProtocol 协议
 * @param ucAddr 地址
 * @param pvChildData 继承者的私有数据
 * @return 返回值
 */
int32_t WitInit(wit_gyro_t handle, uint32_t uiProtocol, uint8_t ucAddr, void *pvChildData)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
	if(uiProtocol > WIT_PROTOCOL_905x_CAN)return WIT_HAL_INVAL;
    p_handle->uiProtoclo = uiProtocol;
    p_handle->ucAddr = ucAddr;
    p_handle->uiWitDataCnt = 0;
    p_handle->pvChildData = pvChildData;
    return WIT_HAL_OK;
}

/**
 * @brief 销毁句柄
 * @param handle 句柄
 */
void WitDeInit(wit_gyro_t handle)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    p_handle->p_WitSerialWriteFunc = NULL;
    p_handle->p_WitI2cWriteFunc = NULL;
    p_handle->p_WitI2cReadFunc = NULL;
    p_handle->p_WitCanWriteFunc = NULL;
    p_handle->p_WitRegUpdateCbFunc = NULL;
	p_handle->p_WitDelaymsFunc = NULL;
    p_handle->ucAddr = 0xff;
    p_handle->uiWitDataCnt = 0;
    p_handle->uiProtoclo = 0;
}

/**
 * @brief 向句柄中注册延时函数
 * @param handle 句柄
 * @param delayms_func 延迟函数
 * @return 返回值
 */
int32_t WitDelayMsRegister(wit_gyro_t handle,DelaymsCb delayms_func)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(!delayms_func)return WIT_HAL_INVAL;
    p_handle->p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

/**
 * @brief 检查范围
 * @param sTemp 值
 * @param sMin 最小值
 * @param sMax 最大值
 * @return 返回值
 */
char CheckRange(short sTemp,short sMin,short sMax)
{
    if ((sTemp>=sMin)&&(sTemp<=sMax)) return 1;
    else return 0;
}


/*Acceleration calibration demo*/
int32_t WitStartAccCali(wit_gyro_t handle)
{
/*
	First place the equipment horizontally, and then perform the following operations
*/
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
	uint8_t ucBuff[3];
	if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61)
	{
	   if(p_handle->p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
       ucBuff[0] = 0xFF;
       ucBuff[1] = 0xAA;
       ucBuff[2] = 0x67;
       p_handle->p_WitSerialWriteFunc(handle,ucBuff, 3);
	   return WIT_HAL_OK;
	}
	if(WitWriteReg(handle,KEY, KEY_UNLOCK) != WIT_HAL_OK) return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, CALSW, CALGYROACC) != WIT_HAL_OK) return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


int32_t WitStopAccCali(wit_gyro_t handle)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
	if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61) return WIT_HAL_INVAL;
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


/*Magnetic field calibration*/
int32_t WitStartMagCali(wit_gyro_t handle)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
	if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61) return WIT_HAL_INVAL;
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, CALSW, CALMAGMM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


int32_t WitStopMagCali(wit_gyro_t handle)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
	if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61) return WIT_HAL_INVAL;
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


/*change Band*/
int32_t WitSetUartBaud(wit_gyro_t handle, int32_t uiBaudIndex)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
	uint8_t ucBuff[3]; 
	if(!CheckRange(uiBaudIndex,WIT_BAUD_4800,WIT_BAUD_230400))
	{
		return WIT_HAL_INVAL;
	}
	if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61)
	{
	    if((uiBaudIndex == WIT_BAUD_115200) || (uiBaudIndex == WIT_BAUD_9600))
		  {
		     if(p_handle->p_WitSerialWriteFunc == NULL)return WIT_HAL_EMPTY;
             ucBuff[0] = 0xFF;
             ucBuff[1] = 0xAA;
		     if(uiBaudIndex==WIT_BAUD_115200) ucBuff[2] = 0x63;
		     else if(uiBaudIndex==WIT_BAUD_9600) ucBuff[2] = 0x64;
             p_handle->p_WitSerialWriteFunc(handle,ucBuff, 3);
		     return WIT_HAL_OK;
		  }
		else return WIT_HAL_INVAL;
	}
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


/*change Can Band*/
int32_t WitSetCanBaud(wit_gyro_t handle, int32_t uiBaudIndex)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
	if(!(p_handle->uiProtoclo == WIT_PROTOCOL_CAN || p_handle->uiProtoclo == WIT_PROTOCOL_905x_CAN)) return WIT_HAL_INVAL;
	if(!CheckRange(uiBaudIndex,CAN_BAUD_1000000,CAN_BAUD_3000))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, BAUD, uiBaudIndex) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


/*change Bandwidth*/
int32_t WitSetBandwidth(wit_gyro_t handle, int32_t uiBaudWidth)
{	
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61) return WIT_HAL_INVAL;
	if(!CheckRange(uiBaudWidth,BANDWIDTH_256HZ,BANDWIDTH_5HZ))
	{
		return WIT_HAL_INVAL;
	}
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, BANDWIDTH, uiBaudWidth) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


/*change output rate */
int32_t WitSetOutputRate(wit_gyro_t handle, int32_t uiRate)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61) return WIT_HAL_INVAL;	
	if(!CheckRange(uiRate,RRATE_02HZ,RRATE_NONE))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, RRATE, uiRate) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}


/*change WitSetContent */
int32_t WitSetContent(wit_gyro_t handle, int32_t uiRsw)
{	
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61) return WIT_HAL_INVAL;
	if(!CheckRange(uiRsw,RSW_TIME,RSW_MASK))
	{
		return WIT_HAL_INVAL;
	}
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, RSW, uiRsw) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/* save parameters */
int32_t WitSaveParameter(wit_gyro_t handle)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61) return WIT_HAL_INVAL;
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/* set software reset */
int32_t WitSetForReset(wit_gyro_t handle)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
    if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61) return WIT_HAL_INVAL;
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, SAVE, SAVE_SWRST) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

/* set Angle reference */
int32_t WitCaliRefAngle(wit_gyro_t handle)
{
    wit_gyro_internal_t *p_handle = (wit_gyro_internal_t *)handle;
	if(p_handle->uiProtoclo == WIT_PROTOCOL_JY61) return WIT_HAL_INVAL;
	if(WitWriteReg(handle, KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;
	if(p_handle->p_WitDelaymsFunc != NULL)
	{
	   p_handle->p_WitDelaymsFunc(20);
	}
	else return WIT_HAL_EMPTY;
	if(WitWriteReg(handle, CALSW, CALREFANGLE) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
