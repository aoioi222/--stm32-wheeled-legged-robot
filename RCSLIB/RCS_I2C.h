
#ifndef _RCS_I2C_H_
#define _RCS_I2C_H_

#include "bsp.h"
#include "RCS_Types.h"
#include "rcs.h"
#define BUFFER_SIZE 10

extern volatile uint8_t txBuffer_1[BUFFER_SIZE];
extern volatile uint8_t txIndex_1;
extern volatile uint8_t txlength_1;

extern volatile uint8_t rxBuffer_1[BUFFER_SIZE];
extern volatile uint8_t rxIndex_1;
extern volatile uint8_t rxlength_1;

extern volatile uint8_t txBuffer_2[BUFFER_SIZE];
extern volatile uint8_t txIndex_2;
extern volatile uint8_t txlength_2;

extern volatile uint8_t rxBuffer_2[BUFFER_SIZE];
extern volatile uint8_t rxIndex_2;
extern volatile uint8_t rxlength_2;

void RCS_Init_I2C(I2C_TypeDef *_I2Cx, GPIO_TypeDef *_GPIOx, 
						uint32_t _GPIO_PinX_SCL, uint32_t _GPIO_PinX_SDA, uint16_t address);

bool RCS_I2C_StartSend(I2C_TypeDef *_I2Cx, uint8_t _address);

bool RCS_I2C_StartRequest(I2C_TypeDef *_I2Cx, uint8_t _address, uint8_t _length);

void RCS_I2C2_SetData(uint8_t _number, uint8_t _angle, uint8_t _speed);

uint8_t isI2CBusy(I2C_TypeDef *_I2Cx);

uint8_t isI2CSuccess(I2C_TypeDef *_I2Cx);

#endif  //_RCS_I2C_H_
