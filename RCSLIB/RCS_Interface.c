//@filename: RCS_Interface.c
//@date: 2021-9-6
//@author: 陈志伟
//@brief: 人机界面含4p OLED显示屏iic通讯

#include "RCS_Interface.h"
#include "codetab.h"

/*
OLED路线信号
*/
#define ROUTE1_GPIO									 GPIOC
#define ROUTE1_PIN							GPIO_Pin_0

#define ROUTE2_GPIO									GPIOC
#define ROUTE2_PIN							GPIO_Pin_1

#define ROUTE3_GPIO									GPIOC
#define ROUTE3_PIN							GPIO_Pin_2

#define ROUTE4_GPIO									GPIOC
#define ROUTE4_PIN							GPIO_Pin_3


//抽象化IO操作
static GPIO_TypeDef*				SCL_GPIO;
static GPIO_TypeDef*				SDA_GPIO;
static uint16_t 						SCL_Pin;
static uint16_t							SDA_Pin;
#define SDA_IN()  {SDA_GPIO->MODER&=~(3<<(GetRCS_PINnum(SDA_Pin)*2));SDA_GPIO->MODER|=0<<GetRCS_PINnum(SDA_Pin)*2;}	
#define SDA_OUT() {SDA_GPIO->MODER&=~(3<<(GetRCS_PINnum(SDA_Pin)*2));SDA_GPIO->MODER|=1<<GetRCS_PINnum(SDA_Pin)*2;} 
#define IIC_SCL    BIT_ADDR(GetRCS_GPIO_ODR_Addr(SCL_GPIO),(GetRCS_PINnum(SCL_Pin))) //SCL
#define IIC_SDA    BIT_ADDR(GetRCS_GPIO_ODR_Addr(SDA_GPIO),(GetRCS_PINnum(SDA_Pin))) //SDA	 
#define READ_SDA   BIT_ADDR(GetRCS_GPIO_IDR_Addr(SDA_GPIO),(GetRCS_PINnum(SDA_Pin))) //SDA	 
static uint8_t OLED_GRAM[144][8];
/****************静态函数****************/
static void IIC_Start(void);				//发送IIC开始信号
static void IIC_Stop(void);	  			//发送IIC停止信号
static void IIC_Send_Byte(u8 txd);	//IIC发送一个字节
static u8 IIC_Wait_Ack(void); 			//IIC等待ACK信号
static void WriteCmd(u8 command);		//命令
//static int num_a,num_b,num_c,num_d;

/****************************************/


/**
	@name: RouteChoose
	@brief: 路线选择初始化函数（有硬件开关控制）
	@return:第几条路线
**/
int RouteChoose(void)
{
	int route_num = 0;
	if(!RCS_GPIO_Read(ROUTE1_GPIO,ROUTE1_PIN))
			route_num+=1;
	if(!RCS_GPIO_Read(ROUTE2_GPIO,ROUTE2_PIN))
			route_num+=2;
	if(!RCS_GPIO_Read(ROUTE3_GPIO,ROUTE3_PIN))
			route_num+=4;
	if(!RCS_GPIO_Read(ROUTE4_GPIO,ROUTE4_PIN))
			route_num+=8;
	return route_num;
}

/**
	@name: RouteChoose_Init
	@brief: 路线选择初始化函数（有硬件开关控制）
	@return:第几条路线
**/
void Routechoose_Init(void)   //人机交互初始化
{
	RCS_GPIO_Input_Init(ROUTE1_GPIO, ROUTE1_PIN);
	RCS_GPIO_Input_Init(ROUTE2_GPIO, ROUTE2_PIN);
	RCS_GPIO_Input_Init(ROUTE3_GPIO, ROUTE3_PIN);
	RCS_GPIO_Input_Init(ROUTE4_GPIO, ROUTE4_PIN);

//	num_a=1-RCS_GPIO_Read(ROUTE1_GPIO,ROUTE1_PIN);
//	num_b=1-RCS_GPIO_Read(ROUTE2_GPIO,ROUTE2_PIN);
//	num_c=1-RCS_GPIO_Read(ROUTE3_GPIO,ROUTE3_PIN);
//	num_d=1-RCS_GPIO_Read(ROUTE4_GPIO,ROUTE4_PIN);
}


/**
	@name: OLED_LED_Init
	@brief: 人机界面上LED初始化并闪烁3秒
**/
void OLED_LED_Init(void)
{
	RCS_GPIO_Output_Init(OLED_LED1_GPIO, OLED_LED1_PIN);
	RCS_GPIO_Output_Init(OLED_LED2_GPIO, OLED_LED2_PIN);
	
	for(int i=0;i<3;i++)
	{
		RCS_GPIO_Set(OLED_LED1_GPIO,OLED_LED1_PIN);
		RCS_GPIO_Set(OLED_LED2_GPIO,OLED_LED2_PIN);
		delay_ms(500);
		RCS_GPIO_Reset(OLED_LED1_GPIO,OLED_LED1_PIN);
		RCS_GPIO_Reset(OLED_LED2_GPIO,OLED_LED2_PIN);
		delay_ms(500);
	}
}

/**
	@name: OLED_LED_Show
	@brief: 使人机界面上OLED D1闪烁
**/
void OLED_LED_Show(void)
{
	RCS_GPIO_Set(OLED_LED1_GPIO,OLED_LED1_PIN);
	delay_ms(500);
	RCS_GPIO_Reset(OLED_LED1_GPIO,OLED_LED1_PIN);
	delay_ms(500);
}
 
/**
	@name: OLED_Show_Route
	@brief: OLED显示当前路线号
**/
void OLED_Show_Route(void)
{
	int route_num = 0;
	char str_num[5];
	route_num = 1;//RouteChoose();
	Int2Str(route_num,str_num);
//	OLED_ShowStr(0, 0, str, 1);
	OLED_ShowStr(0, 0, str_num, 2);
}	
/**
	@name: IIC_Start
	@brief: IIC开始产生信号
**/
static void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  

/**
	@name: IIC_Stop
	@brief: IIC停止产生信号
**/
static void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);							   	
}


/**
	@name: IIC_Wait_Ack
	@brief: IIC等待应答信号到来
	@return:1，接收应答失败  0，接收应答成功
**/
static u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
				 				     
/**
	@name: IIC_Send_Byte
	@brief: IIC发送一个字节
	@return:1，有应答  0，无应答
**/
static void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    

/**
	@name: WriteCmd
	@brief: 发送命令
**/
static void WriteCmd(u8 command)
{
    IIC_Start();
    IIC_Send_Byte(0x78);//OLED地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0x00);//寄存器地址
    IIC_Wait_Ack();
    IIC_Send_Byte(command);
    IIC_Wait_Ack();
    IIC_Stop();
}

/**
	@name: WriteDat
	@brief: 改写数据
**/
void WriteDat(u8 data)
{
    IIC_Start();
    IIC_Send_Byte(0x78);//OLED地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0x40);//寄存器地址
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    IIC_Wait_Ack();
    IIC_Stop();
}


/**
	@name: OLED_Init
	@brief:4Pin OLED初始化
	@param:GPIO_TypeDef* SCL_GPIOx      	 时钟线GPIO
	@param:uint16_t SCL_Pin_x							 时钟线Pin
	@param:GPIO_TypeDef* SDA_GPIOx      	 数据线GPIO
	@param:uint16_t SCL_Pin_x							 数据线Pin
**/
void OLED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	// assert_param(IS_GPIO_ALL_PERIPH(SCL_GPIOx));
	// assert_param(IS_GPIO_PIN(SCL_Pin_x));
	// assert_param(IS_GPIO_ALL_PERIPH(SDA_GPIOx));
	// assert_param(IS_GPIO_PIN(SDA_Pin_x));
	
	SCL_GPIO = OLED_SCL_GPIO;						//抽象参数具体化
	SCL_Pin = OLED_SCL_PIN;
	SDA_GPIO = OLED_SDA_GPIO;
	SDA_Pin = OLED_SDA_PIN;
	
  RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(OLED_SCL_GPIO)|GetRCS_RCC_AHB1Periph_GPIO(OLED_SDA_GPIO), ENABLE);//使能GPIOB时钟

  GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(OLED_SCL_GPIO, &GPIO_InitStructure);//初始化
	
	IIC_SCL=1;
	IIC_SDA=1;
	
	delay_ms(100); //这里的延时很重要

  WriteCmd(0xAE); //display of
	WriteCmd(0x20);	//Set Memory Addressing Mode	
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //亮度调节 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
	delay_ms(10);
	OLED_CLS();
}


 /**
  * @brief  OLED_ON，将OLED从休眠中唤醒
  * @param  无
    * @retval 无
  */
void OLED_ON(void)
{
    WriteCmd(0X8D);  //设置电荷泵
    WriteCmd(0X14);  //开启电荷泵
    WriteCmd(0XAF);  //OLED唤醒
}

/**
	@name: OLED_SetPos
	@brief:设置起始点坐标
	@param:unsigned char x						x坐标，0~127
	@param:unsigned char y						y页，0~7
**/
void OLED_SetPos(unsigned char x, unsigned char y) 
{ 
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);
	WriteCmd((x&0x0f)|0x01);
}

/**
	@name: OLED_Fill
	@brief:OLED填充屏幕
	@param:unsigned char fill_Data			屏幕填充的数据
**/
void OLED_Fill(unsigned char fill_Data)//全屏填充
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				WriteDat(fill_Data);
			}
	}
}

/**
	@name: OLED_CLS
	@brief:清屏
**/
void OLED_CLS(void)
{
	OLED_Fill(0x00);
}

/**
	@name: OLED_OFF
	@brief:OLED休眠模式
**/
void OLED_OFF(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X10);  //关闭电荷泵
	WriteCmd(0XAE);  //OLED休眠
}

/**
	@name: OLED_DrowPoint
	@brief:画点
	+---------------→x
	|
	|	
	|					·
	|
	↓y
	@param:unsigned char x			0~127			x方向，向右为正
	@param:unsigned char y			0~63			y方向，向下为正
	@param:uint8_t drow					0绘制点，1清除点
**/
void OLED_DrowPoint(unsigned char x, unsigned char y,uint8_t drow)
{
	uint8_t floor_y,m,data;
	floor_y = y/8;
	m = y%8;
	data= (uint8_t)(0x01<<m);
	if(drow)
		OLED_GRAM[x][floor_y]=(~data)&OLED_GRAM[x][floor_y];
	else
		OLED_GRAM[x][floor_y]=data|OLED_GRAM[x][floor_y];
	OLED_SetPos(x,floor_y);
	WriteDat(OLED_GRAM[x][floor_y]);
}



//--------------------------------------------------------------
// Prototype      : void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
// Calls          : 
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); ch[] -- 要显示的字符串; TextSize -- 字符大小(1:6*8 ; 2:8*16)
// Description    : 显示codetab.h中的ASCII字符,有6*8和8*16可选择
//--------------------------------------------------------------
/**
	@name: OLED_ShowStr
	@brief:x,y -- 起始点坐标(x:0~127, y:0~7); ch[] -- 要显示的字符串; TextSize -- 字符大小(1:6*8 ; 2:8*16)
**/
void OLED_ShowStr(unsigned char x, unsigned char y, char ch[], unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;
	
	switch(TextSize)
	{
		case 1:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<6;i++)
					WriteDat(F6x8[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i]);
				OLED_SetPos(x,y+1);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i+8]);
				x += 8;
				j++;
			}
		}break;
	}
}