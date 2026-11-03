//@filename: OLED.c
//@date: 2019-07-21
//@author: 闫锐
//@brief: 6针脚和7针脚的OLED显示屏
#include "RCS_OLED.h"

#ifdef OLED_6PIN

/*     -------------
       |   Xm=127  |
       |   Ym=7    |
       |竖着存数据 |
       |   819     |
       -------------     */

void RCS_OLED_Init(void)
{
    SetGpioOutput(OLED_GPIO, OLED_DC_PIN);
    SetGpioOutput(OLED_GPIO, OLED_RST_PIN);
    SetGpioOutput(OLED_GPIO, OLED_SDA_PIN);
    SetGpioOutput(OLED_GPIO, OLED_SCL_PIN);

    OLED_SCL_H;
    OLED_RST_L;
    OSTimeDly(100);
    OLED_RST_H;

    RCS_OLED_WR_CMD(0xae);//关闭oled显示
    RCS_OLED_WR_CMD(0x00);//设置低位光标col位置
    RCS_OLED_WR_CMD(0x10);//设置高位光标col位置
    RCS_OLED_WR_CMD(0x40);//设置开始行位置（0->63）=>（0x40->7f）
    RCS_OLED_WR_CMD(0x81);//设置对比度
    RCS_OLED_WR_CMD(0xcf);//设对比度为207
    RCS_OLED_WR_CMD(0xa1);//Set Segment Re-map（这个很难表达，自己测试下就知道）
    RCS_OLED_WR_CMD(0xc8);//Set COM/Row Scan Direction  （这个很难表达，自己测试下就知道）
    RCS_OLED_WR_CMD(0xa6);//设置正常显示模式 0xa7为反显
    RCS_OLED_WR_CMD(0xa8);//--set multiplex ratio(1 to 64)
    RCS_OLED_WR_CMD(0x3f);//--1/64 duty
    RCS_OLED_WR_CMD(0xd3);//-set display offset    Shift Mapping RAM Counter (0x00~0x3F)
    RCS_OLED_WR_CMD(0x00);//-not offset
    RCS_OLED_WR_CMD(0xd5);//设置显示时钟（震荡频率，divide比例）
    RCS_OLED_WR_CMD(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
    RCS_OLED_WR_CMD(0xd9);//--set pre-charge period
    RCS_OLED_WR_CMD(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    RCS_OLED_WR_CMD(0xda);//--set com pins hardware configuration
    RCS_OLED_WR_CMD(0x12);
    RCS_OLED_WR_CMD(0xdb);//--set vcomh
    RCS_OLED_WR_CMD(0x40);//Set VCOM Deselect Level
    RCS_OLED_WR_CMD(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
    RCS_OLED_WR_CMD(0x02);
    RCS_OLED_WR_CMD(0x8d);//--set Charge Pump enable/disable
    RCS_OLED_WR_CMD(0x14);//--set(0x10) disable
    RCS_OLED_WR_CMD(0xa4);// Disable Entire Display On (0xa4/0xa5)
    RCS_OLED_WR_CMD(0xaf);//--turn on oled panel
    RCS_OLED_FULL(0x00);
    RCS_OLED_Set_Pos(0, 0);
}

void RCS_OLED_WR_DATA(uint8_t data)
{
    uint8_t i = 8;
    OLED_DC_H;
    _DELAY;
    OLED_SCL_L;
    _DELAY;
    while (i--)
    {
        if (data & 0x80)OLED_SDA_H;
        else OLED_SDA_L;
        _DELAY;
        OLED_SCL_H;
        _DELAY;
        OLED_SCL_L;
        _DELAY;
        data <<= 1;
    }
}

void RCS_OLED_WR_CMD(uint8_t cmd)
{
    uint8_t i = 8;
    OLED_DC_L;
    _DELAY;
    OLED_SCL_L;
    _DELAY;
    while (i--)
    {
        if (cmd & 0x80)OLED_SDA_H;
        else OLED_SDA_L;
        _DELAY;
        OLED_SCL_H;
        _DELAY;
        OLED_SCL_L;
        _DELAY;
        cmd <<= 1;
    }
}
void RCS_OLED_PutPixel(uint8_t x, uint8_t y)
{
    uint8_t data1;  //data1当前点的数据

    RCS_OLED_Set_Pos(x, y);
    data1 = 0x01 << (y % 8);
    RCS_OLED_WR_CMD(0xb0 + (y >> 3));
    RCS_OLED_WR_CMD(((x & 0xf0) >> 4) | 0x10);
    RCS_OLED_WR_CMD((x & 0x0f) | 0x00);
    RCS_OLED_WR_DATA(data1);
}
void RCS_OLED_Set_Pos(uint8_t x, uint8_t y)
{
    RCS_OLED_WR_CMD(0xb0 + y);
    RCS_OLED_WR_CMD(((x & 0xf0) >> 4) | 0x10);
    RCS_OLED_WR_CMD((x & 0x0f) | 0x01);
}

void RCS_OLED_WR_8X16STR(uint8_t x, uint8_t y, uint8_t *str)//x是列，y为行，一个字符占1行8列
{
    uint8_t ch , i, j;
    for (i = 0; str[i] != '\0'; i++)
    {
        ch = str[i] - 32;
        if (x > 120)
        {
            x = 0;
            y += 2;
        }
        RCS_OLED_Set_Pos(x, y);
        for (j = 0; j < 8; j++)
            RCS_OLED_WR_DATA(F8X16[ch * 16 + j]);
        RCS_OLED_Set_Pos(x, y + 1);
        for (j = 0; j < 8; j++)
            RCS_OLED_WR_DATA(F8X16[ch * 16 + j + 8]);
        x += 8;
    }
}

void RCS_OLED_WR_6X8STR(uint8_t x, uint8_t y, uint8_t *str)
{
    uint8_t ch, i, j;
    for (i = 0; str[i] != '\0'; i++)
    {
        ch = str[i] - 32;
        if (x > 122)
        {
            x = 0;
            y++;
        }
        RCS_OLED_Set_Pos(x, y);
        for (j = 0; j < 6 ; j++)
            RCS_OLED_WR_DATA(F6x8[ch][j]);
        x += 6;
    }
}


void RCS_OLED_FULL(uint8_t ch)
{
    uint8_t i, j;
    for (i = 0; i < 8; i++)
    {
        RCS_OLED_Set_Pos(0, i);
        for (j = 0; j < 128; j++)
            RCS_OLED_WR_DATA(ch);
    }
}

void RCS_OLED_LEFT_SROLL(void)
{
    RCS_OLED_WR_CMD(0x27);
    RCS_OLED_WR_CMD(0x00);
    RCS_OLED_WR_CMD(0x00);
    RCS_OLED_WR_CMD(0x00);
    RCS_OLED_WR_CMD(0x01);
    RCS_OLED_WR_CMD(0x00);
    RCS_OLED_WR_CMD(0xff);
    RCS_OLED_WR_CMD(0x2f);   //0x2f 水平滚动开始，0x2e水平滚动停止
}

void RCS_OLED_RIGHT_SROLL(void)
{
    RCS_OLED_WR_CMD(0x26);
    RCS_OLED_WR_CMD(0x00);
    RCS_OLED_WR_CMD(0x00);
    RCS_OLED_WR_CMD(0x04);
    RCS_OLED_WR_CMD(0x01);
    RCS_OLED_WR_CMD(0x00);
    RCS_OLED_WR_CMD(0xff);
    RCS_OLED_WR_CMD(0x2f);
}

void RCS_OLED_UP_DOWN_SROLL(void)
{
    RCS_OLED_WR_CMD(0x29);   //[29h]: Vertical and Right Horizontal Scroll;[2Ah]: Vertical and Left Horizontal Scroll
    RCS_OLED_WR_CMD(0x00);   //Dummy byte(Set as 00h)
    RCS_OLED_WR_CMD(0x00);   //Define start page address(0~7)
    RCS_OLED_WR_CMD(0x01);   //Set time interval between each scroll step in terms of  frame frequency
    RCS_OLED_WR_CMD(0x01);   //Define end page address(0~7),The value of start page address must be larger or equal to end page address
    RCS_OLED_WR_CMD(0x00);   //Vertical scrolling offset(0~63)
    RCS_OLED_WR_CMD(0x2f);   //Activate scroll
}

void RCS_OLED_Convert_Num2Str(int num, uint8_t *str)
{
    uint8_t tmp[10];  //这里不能改成uint8_t * tmp ，自己体会原因。
    int j = 0, i = 0;
    if (num < 0)
    {
        str[j++] = '-';
        num = -num;
    }
    do
    {
        tmp[i++] = num % 10 + '0';
        num /= 10;
    }
    while (num);
    while (i)
        str[j++] = tmp[--i];
    str[j] = '\0';
}


void RCS_OLED_Convert_Float2Str(float num, uint8_t *str)
{
    sprintf(str, "%.4f", num);
}


#endif

#ifdef OLED_7PIN

//初始化SSD1306
void RCS_OLED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG, ENABLE); //使能PORTA~E,PORTG时钟

    //GPIO初始化设置
    GPIO_InitStructure.GPIO_Pin = OLED_CS_PIN | OLED_DC_PIN | OLED_RST_PIN | OLED_SDIN_PIN | OLED_SCLK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(OLED_GPIO, &GPIO_InitStructure);//初始化

    OLED_RST_Set();
    OSTimeDly(200);
    OLED_RST_Clr();
    OSTimeDly(200);
    OLED_RST_Set();

    RCS_OLED_WR_Byte(0xAE, OLED_CMD); //--turn off oled panel
    RCS_OLED_WR_Byte(0x02, OLED_CMD); //---set low column address
    RCS_OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    RCS_OLED_WR_Byte(0x40, OLED_CMD); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    RCS_OLED_WR_Byte(0x81, OLED_CMD); //--set contrast control register
    RCS_OLED_WR_Byte(0xff, OLED_CMD); // Set SEG Output Current Brightness
    RCS_OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    RCS_OLED_WR_Byte(0xC8, OLED_CMD); //Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    RCS_OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
    RCS_OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    RCS_OLED_WR_Byte(0x3f, OLED_CMD); //--1/64 duty
    RCS_OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset Shift Mapping RAM Counter (0x00~0x3F)
    RCS_OLED_WR_Byte(0x00, OLED_CMD); //-not offset
    RCS_OLED_WR_Byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
    RCS_OLED_WR_Byte(0x80, OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
    RCS_OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period
    RCS_OLED_WR_Byte(0xF1, OLED_CMD); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    RCS_OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
    RCS_OLED_WR_Byte(0x12, OLED_CMD);
    RCS_OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh
    RCS_OLED_WR_Byte(0x40, OLED_CMD); //Set VCOM Deselect Level
    RCS_OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
    RCS_OLED_WR_Byte(0x02, OLED_CMD); //
    RCS_OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
    RCS_OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
    RCS_OLED_WR_Byte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
    RCS_OLED_WR_Byte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7)
    RCS_OLED_WR_Byte(0xAF, OLED_CMD); //--turn on oled panel

    RCS_OLED_WR_Byte(0xAF, OLED_CMD); /*display ON*/
    RCS_OLED_Display_Clear();
    RCS_OLED_Set_Pos(0, 0);
}

void RCS_OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t dotbyte = 0, pos, bx, temp = 0;
    if (x > 127 || y > 63)return; //?????.
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if (t) dotbyte |= temp;
    else dotbyte &= ~temp;
    RCS_OLED_Set_Pos(x, 63 - y);
    RCS_OLED_WR_Byte(dotbyte, OLED_DATA);
}
void RCS_OLED_DrawPoint2(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t dotbyte = 0, pos, bx, temp = 0;
    if (x > 127 || y > 63)return; //?????.
    //pos=7-y/8;
    //bx=y%8;
    //temp=1<<(7-bx);
    if (t) dotbyte |= temp;
    else dotbyte &= temp;
    RCS_OLED_Set_Pos(x, 63 - y);
    RCS_OLED_WR_Byte(dotbyte, OLED_DATA);
}

void RCS_OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    uint8_t i;
    if (cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();
    OLED_CS_Clr();
    for (i = 0; i < 8; i++)
    {
        OLED_SCLK_Clr();
        if (dat & 0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        OLED_SCLK_Set();
        dat <<= 1;
    }
    OLED_CS_Set();
    OLED_DC_Set();
}

void RCS_OLED_Set_Pos(uint8_t x, uint8_t y)
{
    y = y / 8;
    RCS_OLED_WR_Byte(0xb0 + y, OLED_CMD);
    RCS_OLED_WR_Byte((((x + 2) & 0xf0) >> 4) | 0x10, OLED_CMD);
    RCS_OLED_WR_Byte(((x + 2) & 0x0f), OLED_CMD);
}
//开启OLED显示
void RCS_OLED_Display_On(void)
{
    RCS_OLED_WR_Byte(0X8D, OLED_CMD); //SET DCDC命令
    RCS_OLED_WR_Byte(0X14, OLED_CMD); //DCDC ON
    RCS_OLED_WR_Byte(0XAF, OLED_CMD); //DISPLAY ON
}
//关闭OLED显示
void RCS_OLED_Display_Off(void)
{
    RCS_OLED_WR_Byte(0X8D, OLED_CMD); //SET DCDC命令
    RCS_OLED_WR_Byte(0X10, OLED_CMD); //DCDC OFF
    RCS_OLED_WR_Byte(0XAE, OLED_CMD); //DISPLAY OFF
}
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void RCS_OLED_Display_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        RCS_OLED_WR_Byte (0xb0 + i, OLED_CMD); //设置页地址（0~7）
        RCS_OLED_WR_Byte (0x02, OLED_CMD);     //设置显示位置―列低地址
        RCS_OLED_WR_Byte (0x10, OLED_CMD);     //设置显示位置―列高地址
        for (n = 0; n < 128; n++)RCS_OLED_WR_Byte(0, OLED_DATA);
    } //更新显示
}

uint32_t RCS_OLED_Pow(uint8_t m, uint8_t n)
{
    uint32_t ret = 1;
    while (n--)
        ret *= m;
    return ret;
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//mode:0,反白显示;1,正常显示
//size:选择字体 16/12
void RCS_OLED_Display_Char(uint8_t x, uint8_t y, uint8_t chr)
{
    uint8_t c = 0, i = 0;
    c = chr - ' '; //得到偏移后的值
    if (x > Max_Column - 1) {x = 0; y = y + 2;}
    if (SIZE == 16)
    {
        RCS_OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            RCS_OLED_WR_Byte(F8X16[c * 16 + i], OLED_DATA);
        RCS_OLED_Set_Pos(x, y + 8);
        for (i = 0; i < 8; i++)
            RCS_OLED_WR_Byte(F8X16[c * 16 + i + 8], OLED_DATA);
    }
    else {
        RCS_OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 6; i++)
            RCS_OLED_WR_Byte(F6x8[c][i], OLED_DATA);

    }
}

//显示1个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:模式   0,填充模式;1,叠加模式
//num:数值(0~4294967295);
void RCS_OLED_Display_Num(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / RCS_OLED_Pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                RCS_OLED_Display_Char(x + (size / 2)*t, y, ' ');
                continue;
            } else enshow = 1;

        }
        RCS_OLED_Display_Char(x + (size / 2)*t, y, temp + '0');
    }
}
//显示一个字符号串
void RCS_OLED_Display_String(uint8_t x, uint8_t y, uint8_t *chr)
{
    uint8_t j = 0;
    while (chr[j] != '\0')
    {
        RCS_OLED_Display_Char(x, y, chr[j]);
        x += 8;
        if (x > 120) {x = 0; y += 2;}
        j++;
    }
}

/***********功能描述：显示显示BMP图片128×64起始点坐标(x,y),x的范围0～127，y为页的范围0～7*****************/
void RCS_OLED_Display_Picture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t BMP[])
{
    unsigned int j = 0;
    uint8_t x, y;

    if (y1 % 8 == 0) y = y1 / 8;
    else y = y1 / 8 + 1;
    for (y = y0; y < y1; y++)
    {
        RCS_OLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            RCS_OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}

#endif
