#include "rcs.h"
void InitSpi(SPI_TypeDef *_s, uint16_t _mode,
			 GPIO_TypeDef *_clkG, uint32_t _clkP,
			 GPIO_TypeDef *_mosiG, uint32_t _mosiP,
			 GPIO_TypeDef *_misoG, uint32_t _misoP,
			 uint16_t _CPOL,uint16_t _CPHA)
{
	assert_param(IS_SPI_CPOL(_CPOL));
	assert_param(IS_SPI_CPHA(_CPHA));
	if (_s == SPI1)
	{
		RCC_APB2PeriphClockCmd(GetRCS_RCC_APB2Periph_SPI(_s), ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd(GetRCS_RCC_APB1Periph_SPI(_s), ENABLE);
	}

	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_clkG), ENABLE);
	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_mosiG), ENABLE);
	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_misoG), ENABLE);

	GPIO_PinAFConfig(_clkG, GetRCS_GPIO_PinSource(_clkP), GetRCS_GPIO_AF_SPI(_s));
	GPIO_PinAFConfig(_mosiG, GetRCS_GPIO_PinSource(_mosiP), GetRCS_GPIO_AF_SPI(_s));
	GPIO_PinAFConfig(_misoG, GetRCS_GPIO_PinSource(_misoP), GetRCS_GPIO_AF_SPI(_s));

	GPIO_InitTypeDef GPIO_Struct;

	GPIO_StructInit(&GPIO_Struct);
	GPIO_Struct.GPIO_Pin = _clkP;
	GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef  OD_OUTPUT
	GPIO_Struct.GPIO_OType = GPIO_OType_OD;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
#else
	GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_UP;
#endif
	GPIO_Init(_clkG, &GPIO_Struct);

	GPIO_StructInit(&GPIO_Struct);
	GPIO_Struct.GPIO_Pin = _mosiP;
	GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef  OD_OUTPUT
	GPIO_Struct.GPIO_OType = GPIO_OType_OD;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
#else
	GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_UP;
#endif
	GPIO_Init(_mosiG, &GPIO_Struct);

	GPIO_StructInit(&GPIO_Struct);
	GPIO_Struct.GPIO_Pin = _misoP;
	GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef  OD_OUTPUT
	GPIO_Struct.GPIO_OType = GPIO_OType_OD;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
#else
	GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	GPIO_Struct.GPIO_PuPd = GPIO_PuPd_UP;
#endif
	GPIO_Init(_misoG, &GPIO_Struct);

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

	SPI_InitStructure.SPI_Mode = _mode;

	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;

	SPI_InitStructure.SPI_CPOL = _CPOL;
	SPI_InitStructure.SPI_CPHA = _CPHA;
	if (_mode == SPI_Mode_Master)
	{
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	}
	else
	{
		SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	}
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(_s, &SPI_InitStructure);
	SPI_CalculateCRC(_s,DISABLE);
	SPI_Cmd(_s, ENABLE);
	SPI_CalculateCRC(_s,DISABLE);
}

