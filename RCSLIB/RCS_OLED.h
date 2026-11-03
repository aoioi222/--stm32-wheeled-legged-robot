//@filename: OLED.c
//@date: 2019-07-21
//@author: 闫锐
//@brief: 6针脚和7针脚的OLED显示屏
#ifndef _RCS_OLED_H_
#define _RCS_OLED_H_

#include "rcs.h"


#define   OLED_PIN_CHOSE    0
#if       OLED_PIN_CHOSE == 0
  #define OLED_4PIN
#elif     OLED_PIN_CHOSE == 1
  #define OLED_6PIN
#elif     OLED_PIN_CHOSE == 2    
  #define OLED_7PIN
#endif



#define Max_Column  128
#define Max_Row     64
#define SIZE 16 //显示字体选择

//OLED操作
void RCS_OLED_Init(void);//初始化
void RCS_OLED_Set_Pos(uint8_t x, uint8_t y);//设置坐标

#ifdef OLED_6PIN

#define _DELAY asm volatile ("nop");\
asm volatile ("nop");

void RCS_OLED_WR_6X8STR(uint8_t x, uint8_t y, uint8_t *str);
void RCS_OLED_WR_8X16STR(uint8_t x, uint8_t y, uint8_t *str);
void RCS_OLED_WR_DATA(uint8_t data);
void RCS_OLED_WR_CMD(uint8_t cmd);
void RCS_OLED_FULL(uint8_t ch);//填充
void RCS_OLED_LEFT_SROLL(void);//向左滚动
void RCS_OLED_RIGHT_SROLL(void);//向右滚动
void RCS_OLED_UP_DOWN_SROLL(void);//上下滚动
void RCS_OLED_PutPixel(uint8_t x, uint8_t y);//画像素
void RCS_OLED_Convert_Num2Str(int num, uint8_t *str);
void RCS_OLED_Convert_Float2Str(float num, uint8_t *str);
#endif

#ifdef OLED_7PIN

void RCS_OLED_Display_On(void);//开关显示
void RCS_OLED_Display_Off(void);
void RCS_OLED_Display_Clear(void);//清屏
void RCS_OLED_Display_Char(uint8_t x, uint8_t y, uint8_t str);//显示一个字符
void RCS_OLED_Display_String(uint8_t x, uint8_t y, uint8_t *str);//显示字符串
void RCS_OLED_Display_Num(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void RCS_OLED_Display_Picture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[]);//显示图片
void RCS_OLED_WR_Byte(u8 dat, u8 cmd);
void RCS_OLED_DrawPoint(u8 x, u8 y, u8 t);
void RCS_OLED_DrawPoint2(u8 x, u8 y, u8 t);
uint32_t RCS_OLED_Pow(uint8_t m, uint8_t n);

#endif

#endif

