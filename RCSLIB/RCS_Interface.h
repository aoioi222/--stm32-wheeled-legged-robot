#ifndef _RCS_INTERFACE_H_
#define _RCS_INTERFACE_H_
#include "rcs.h"

#define OLED_ADDRESS	0x78 //通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78

/*
OLED显示屏
 */
#ifdef OLED_6PIN

#define OLED_GPIO			GPIOA

#define OLED_DC_PIN         GPIO_Pin_3
#define OLED_RST_PIN        GPIO_Pin_4
#define OLED_SDA_PIN        GPIO_Pin_5
#define OLED_SCL_PIN        GPIO_Pin_6

#define OLED_DC_L                GPIO_ResetBits(OLED_GPIO,OLED_DC_PIN)
#define OLED_DC_H                GPIO_SetBits(OLED_GPIO,OLED_DC_PIN)
#define OLED_RST_L               GPIO_ResetBits(OLED_GPIO,OLED_RST_PIN)
#define OLED_RST_H               GPIO_SetBits(OLED_GPIO,OLED_RST_PIN)
#define OLED_SDA_L               GPIO_ResetBits(OLED_GPIO,OLED_SDA_PIN)
#define OLED_SDA_H               GPIO_SetBits(OLED_GPIO,OLED_SDA_PIN)
#define OLED_SCL_L               GPIO_ResetBits(OLED_GPIO,OLED_SCL_PIN)
#define OLED_SCL_H               GPIO_SetBits(OLED_GPIO,OLED_SCL_PIN)

#endif

#ifdef OLED_7PIN

#define OLED_GPIO		GPIOG

#define OLED_CS_PIN		GPIO_Pin_3
#define OLED_CS_Clr()  	GPIO_ResetBits(OLED_GPIO,OLED_CS_PIN)//CS
#define OLED_CS_Set()  	GPIO_SetBits(OLED_GPIO,OLED_CS_PIN)

#define OLED_DC_PIN		GPIO_Pin_4
#define OLED_DC_Clr() 	GPIO_ResetBits(OLED_GPIO,OLED_DC_PIN)//DC
#define OLED_DC_Set() 	GPIO_SetBits(OLED_GPIO,OLED_DC_PIN)

#define OLED_RST_PIN	GPIO_Pin_5
#define OLED_RST_Clr() 	GPIO_ResetBits(OLED_GPIO,OLED_RST_PIN)//RES
#define OLED_RST_Set() 	GPIO_SetBits(OLED_GPIO,OLED_RST_PIN)

#define OLED_SDIN_PIN	GPIO_Pin_6
#define OLED_SDIN_Clr() GPIO_ResetBits(OLED_GPIO,OLED_SDIN_PIN)//DATA/D1
#define OLED_SDIN_Set() GPIO_SetBits(OLED_GPIO,OLED_SDIN_PIN)

#define OLED_SCLK_PIN	GPIO_Pin_7
#define OLED_SCLK_Clr() GPIO_ResetBits(OLED_GPIO,OLED_SCLK_PIN)//CLK/D0
#define OLED_SCLK_Set() GPIO_SetBits(OLED_GPIO,OLED_SCLK_PIN)

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

#endif

#ifdef OLED_4PIN

//#define OLED_SCL_GPIO					GPIOB
//#define OLED_SCL_PIN					GPIO_Pin_10
//#define OLED_SDA_GPIO					GPIOB
//#define OLED_SDA_PIN					GPIO_Pin_11

#define OLED_SCL_GPIO					GPIOE
#define OLED_SCL_PIN					GPIO_Pin_9
#define OLED_SDA_GPIO					GPIOE
#define OLED_SDA_PIN					GPIO_Pin_11

#define OLED_LED1_GPIO				GPIOB
#define OLED_LED1_PIN					GPIO_Pin_15
#define OLED_LED2_GPIO				GPIOB
#define OLED_LED2_PIN					GPIO_Pin_14

#endif


int RouteChoose(void);
void Routechoose_Init(void);
void OLED_LED_Init(void);
void OLED_LED_Show(void);
void OLED_Show_Route(void);

void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_DrowPoint(unsigned char x, unsigned char y,uint8_t drow);
void OLED_ShowStr(unsigned char x, unsigned char y, char ch[], unsigned char TextSize);
void WriteDat(uint8_t data);			//数据
#endif
