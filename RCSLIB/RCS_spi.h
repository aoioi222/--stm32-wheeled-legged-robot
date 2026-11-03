#ifndef _RCS_SPI_H_
#define _RCS_SPI_H_

void InitSpi(SPI_TypeDef *_s, uint16_t _mode,
			 GPIO_TypeDef *_clkG, uint32_t _clkP,
			 GPIO_TypeDef *_mosiG, uint32_t _mosiP,
			 GPIO_TypeDef *_misoG, uint32_t _misoP,
			 uint16_t _CPOL,uint16_t _CPHA);
#endif