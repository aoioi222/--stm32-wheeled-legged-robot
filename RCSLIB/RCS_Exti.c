//文件名：  RCS_exti.c
//日期： 2012-08-17
//作者： 郑文楷&&魏闻
//文件说明：外部中断模块封装
//修改历史
//2012-08-17    23:30   柯国霖   修改函数名 参数名 文件名
//2012-08-18    01:42   高震宙   稍微修改了代码格式
//2012-08-18    09:30   柯国霖   同步注释和代码，修改时间格式，建议统一为 yyyy-mm-dd
//2012-08-19    21:48   高震宙   修正了文件名
//2012-08-25    22:00   柯国霖   大量修改,改用函数封装
//2012-10-14    14:00   柯国霖   添加中断优先级的设定
//2012-12-08    17:00   柯国霖   增加入口参数检查，规范注释
#include "RCS_Exti.h"

uint8_t Pin_Difintion_Flag[PIN_NUM]={0};
GPIO_TypeDef *_GPIO_Difinition_Flag[PIN_NUM]={0};
RCS_EXTI_CB EXTI_CB_Function[EXTI_MAX_NUM]={NULL};
uint16_t _pin[EXTI_MAX_NUM]={0};
GPIO_TypeDef *_GPIO[EXTI_MAX_NUM]={0};
uint8_t	pin_last_vol[EXTI_MAX_NUM]={0};
int EXTI_COUNT=0;

void Function_EXTI_0(void);
void Function_EXTI_1(void);
void Function_EXTI_2(void);
void Function_EXTI_3(void);
void Function_EXTI_4(void);
void Function_EXTI_9_5(void);
void Function_EXTI15_10(void);
static FNCT_VOID GetRCS_EXTI_Function_Name(uint32_t _pin);
int GetRCS_EXTI_Pin_Num(uint32_t _pin);


/**
 * @name:RCS_InitEXTI
 * @brief 初始化EXTI工作所需的外设
 * 
 * @param _port      哪一个GPIO
 * @param _pin       哪一个Pin
 * @param prio       中断优先级	例：0b10001000
 * @return           错误码
 *
 * @note						不同GPIO的相同Pin只能同时存在一个引脚被初始化，例如：PA1和PB1同时只能PA1或PB1被初始化
 */
RCS_EXTI_Err_t RCS_InitEXTI(GPIO_TypeDef *_port, uint32_t _pin,uint8_t _priority)
{
	assert_param(IS_GPIO_ALL_PERIPH(_port));
	assert_param(IS_GPIO_PIN(_pin));
	assert_param(IS_EXTI_TRIGGER(_trigger));
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_port), ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure  pin as input floating */
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = _pin;
	GPIO_Init(_port, &GPIO_InitStructure);

	/* Connect EXTI Line topin */
	SYSCFG_EXTILineConfig(GetRCS_EXTI_PortSourceGPIO(_port), GetRCS_EXTI_PinSource(_pin));

	/* Configure EXTI Line */
	EXTI_StructInit(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line = GetRCS_EXTI_Line(_pin);
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	BSP_IntVectSet(GetRCS_BSP_INT_ID_EXTI(_pin), GetRCS_EXTI_Function_Name(_pin));

	NVIC_InitStructure.NVIC_IRQChannel = GetRCS_EXTI_IRQn(_pin);
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = (_priority >> 4) & 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = _priority & 0x0f;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	if(Pin_Difintion_Flag[GetRCS_EXTI_Pin_Num(_pin)]==1)			
	{
		_GPIO_Difinition_Flag[GetRCS_EXTI_Pin_Num(_pin)]=_port;
		return RCS_EXTI_REDIFINITION;//对应pin已经被初始化过
	}
	else
	{	
		Pin_Difintion_Flag[GetRCS_EXTI_Pin_Num(_pin)]=1;
		_GPIO_Difinition_Flag[GetRCS_EXTI_Pin_Num(_pin)]=_port;
		return RCS_EXTI_OK;//初始化完成
	}
}


/**
 * @brief 向EXTI通道注册回调函数
 * 
 * @param GPIOx        哪一个GPIO
 * @param pin          哪一个Pin
 * @param RCS_EXTI_CB  回调函数指针
 * @return             错误码	或是	对应引脚上此次注册前已有的回调函数的数量
 * 
 * @note               允许在同一个引脚上注册多个回调函数。例如在PA1上注册两个回调函数，则当PA1出现上升沿时，两个回调函数均会被调用。
 */

RCS_EXTI_Err_t RCS_EXTI_Register_callback(GPIO_TypeDef* GPIOx,uint16_t pin,RCS_EXTI_CB cb)
{
	int Pin_CB_Count=0;
	if(EXTI_COUNT>=EXTI_MAX_NUM)	return RCS_EXTI_FULL;//注册中断函数数量超过了允许的EXTI_MAX_NUM
	if(Pin_Difintion_Flag[GetRCS_EXTI_Pin_Num(pin)]!=1||_GPIO_Difinition_Flag[GetRCS_EXTI_Pin_Num(pin)]!=GPIOx)	
		return RCS_EXTI_Register_callback_Error;//对应的pin没有进行中断初始化
	for(EXTI_COUNT;EXTI_COUNT<EXTI_MAX_NUM;EXTI_COUNT++)
	{
		if(EXTI_CB_Function[EXTI_COUNT]==NULL)	
		{
			EXTI_CB_Function[EXTI_COUNT]=cb;
			_GPIO[EXTI_COUNT]=GPIOx;
			_pin[EXTI_COUNT]=pin;
			pin_last_vol[EXTI_COUNT]=RCS_GPIO_Read(GPIOx,pin);
			for(int middle=0;middle<EXTI_COUNT;middle++)
			{
				if(_GPIO[middle]==GPIOx	&&	_pin[middle]==pin)	Pin_CB_Count++;
			}
			if(Pin_CB_Count==0)	return	RCS_EXTI_OK;
			else if(Pin_CB_Count>0)	return	Pin_CB_Count;
		}
	}
}

void SetGpioOutput(GPIO_TypeDef *_port, uint32_t _pin)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_port), ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = _pin ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(_port, &GPIO_InitStructure);
}

void SetGpioInput(GPIO_TypeDef *_port, uint32_t _pin)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(GetRCS_RCC_AHB1Periph_GPIO(_port), ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = _pin ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(_port, &GPIO_InitStructure);
}


static FNCT_VOID GetRCS_EXTI_Function_Name(uint32_t _pin)
{
	switch (_pin)
	{
	case GPIO_Pin_0: return Function_EXTI_0;
	case GPIO_Pin_1: return Function_EXTI_1;
	case GPIO_Pin_2: return Function_EXTI_2;
	case GPIO_Pin_3: return Function_EXTI_3;
	case GPIO_Pin_4: return Function_EXTI_4;
	case GPIO_Pin_5: return Function_EXTI_9_5;
	case GPIO_Pin_6: return Function_EXTI_9_5;
	case GPIO_Pin_7: return Function_EXTI_9_5;
	case GPIO_Pin_8: return Function_EXTI_9_5;
	case GPIO_Pin_9: return Function_EXTI_9_5;
	case GPIO_Pin_10: return Function_EXTI15_10;
	case GPIO_Pin_11: return Function_EXTI15_10;
	case GPIO_Pin_12: return Function_EXTI15_10;
	case GPIO_Pin_13: return Function_EXTI15_10;
	case GPIO_Pin_14: return Function_EXTI15_10;
	case GPIO_Pin_15: return Function_EXTI15_10;
	}
}
int GetRCS_EXTI_Pin_Num(uint32_t _pin)
{
	switch (_pin)
	{
	case GPIO_Pin_0: return 0;
	case GPIO_Pin_1: return 1;
	case GPIO_Pin_2: return 2;
	case GPIO_Pin_3: return 3;
	case GPIO_Pin_4: return 4;
	case GPIO_Pin_5: return 5;
	case GPIO_Pin_6: return 6;
	case GPIO_Pin_7: return 7;
	case GPIO_Pin_8: return 8;
	case GPIO_Pin_9: return 9;
	case GPIO_Pin_10: return 10;
	case GPIO_Pin_11: return 11;
	case GPIO_Pin_12: return 12;
	case GPIO_Pin_13: return 13;
	case GPIO_Pin_14: return 14;
	case GPIO_Pin_15: return 15;
	}
	return -1;
}
void Function_EXTI_0(void)
{
	RCS_EXTI_CURRENT_TRIGGER Current_Trigger; 
	int EXTI_0_Count_NUM=0; 
	while(EXTI_CB_Function[EXTI_0_Count_NUM]!=NULL)
	{
		if(RCS_GPIO_Read(_GPIO[EXTI_0_Count_NUM],_pin[EXTI_0_Count_NUM])!=pin_last_vol[EXTI_0_Count_NUM] 
				&&_pin[EXTI_0_Count_NUM]==GPIO_Pin_0)
		{
			pin_last_vol[EXTI_0_Count_NUM]=RCS_GPIO_Read(_GPIO[EXTI_0_Count_NUM],_pin[EXTI_0_Count_NUM]);
			if(pin_last_vol[EXTI_0_Count_NUM]==1)	Current_Trigger=RCS_EXTI_RISE;
			else if(pin_last_vol[EXTI_0_Count_NUM]==0)	Current_Trigger=RCS_EXTI_FALL;
			
			EXTI_CB_Function[EXTI_0_Count_NUM](_GPIO[EXTI_0_Count_NUM],_pin[EXTI_0_Count_NUM],Current_Trigger);
		}
		EXTI_0_Count_NUM++;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(GPIO_Pin_0));
}
void Function_EXTI_1(void)
{
	RCS_EXTI_CURRENT_TRIGGER Current_Trigger; 
	int EXTI_1_Count_NUM=0; 
	while(EXTI_CB_Function[EXTI_1_Count_NUM]!=NULL)
	{
		if(RCS_GPIO_Read(_GPIO[EXTI_1_Count_NUM],_pin[EXTI_1_Count_NUM])!=pin_last_vol[EXTI_1_Count_NUM]
				&&_pin[EXTI_1_Count_NUM]==GPIO_Pin_1)
		{
			pin_last_vol[EXTI_1_Count_NUM]=RCS_GPIO_Read(_GPIO[EXTI_1_Count_NUM],_pin[EXTI_1_Count_NUM]);
			if(pin_last_vol[EXTI_1_Count_NUM]==1)	Current_Trigger=RCS_EXTI_RISE;
			else if(pin_last_vol[EXTI_1_Count_NUM]==0)	Current_Trigger=RCS_EXTI_FALL;
			
			EXTI_CB_Function[EXTI_1_Count_NUM](_GPIO[EXTI_1_Count_NUM],_pin[EXTI_1_Count_NUM],Current_Trigger);
		}
		EXTI_1_Count_NUM++;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(GPIO_Pin_1));
}

void Function_EXTI_2(void)
{
	RCS_EXTI_CURRENT_TRIGGER Current_Trigger; 
	int EXTI_2_Count_NUM=0; 
	while(EXTI_CB_Function[EXTI_2_Count_NUM]!=NULL)
	{
		if(RCS_GPIO_Read(_GPIO[EXTI_2_Count_NUM],_pin[EXTI_2_Count_NUM])!=pin_last_vol[EXTI_2_Count_NUM]
				&&_pin[EXTI_2_Count_NUM]==GPIO_Pin_2)
		{
			pin_last_vol[EXTI_2_Count_NUM]=RCS_GPIO_Read(_GPIO[EXTI_2_Count_NUM],_pin[EXTI_2_Count_NUM]);
			if(pin_last_vol[EXTI_2_Count_NUM]==1)	Current_Trigger=RCS_EXTI_RISE;
			else if(pin_last_vol[EXTI_2_Count_NUM]==0)	Current_Trigger=RCS_EXTI_FALL;
			
			EXTI_CB_Function[EXTI_2_Count_NUM](_GPIO[EXTI_2_Count_NUM],_pin[EXTI_2_Count_NUM],Current_Trigger);
		}
		EXTI_2_Count_NUM++;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(GPIO_Pin_2));
}
void Function_EXTI_3(void)
{
	RCS_EXTI_CURRENT_TRIGGER Current_Trigger; 
	int EXTI_3_Count_NUM=0; 
	while(EXTI_CB_Function[EXTI_3_Count_NUM]!=NULL)
	{
		if(RCS_GPIO_Read(_GPIO[EXTI_3_Count_NUM],_pin[EXTI_3_Count_NUM])!=pin_last_vol[EXTI_3_Count_NUM]
				&&_pin[EXTI_3_Count_NUM]==GPIO_Pin_3)
		{
			pin_last_vol[EXTI_3_Count_NUM]=RCS_GPIO_Read(_GPIO[EXTI_3_Count_NUM],_pin[EXTI_3_Count_NUM]);
			if(pin_last_vol[EXTI_3_Count_NUM]==1)	Current_Trigger=RCS_EXTI_RISE;
			else if(pin_last_vol[EXTI_3_Count_NUM]==0)	Current_Trigger=RCS_EXTI_FALL;
			
			EXTI_CB_Function[EXTI_3_Count_NUM](_GPIO[EXTI_3_Count_NUM],_pin[EXTI_3_Count_NUM],Current_Trigger);
		}
		EXTI_3_Count_NUM++;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(GPIO_Pin_3));
}
void Function_EXTI_4(void)
{
	RCS_EXTI_CURRENT_TRIGGER Current_Trigger; 
	int EXTI_4_Count_NUM=0; 
	while(EXTI_CB_Function[EXTI_4_Count_NUM]!=NULL)
	{
		if(RCS_GPIO_Read(_GPIO[EXTI_4_Count_NUM],_pin[EXTI_4_Count_NUM])!=pin_last_vol[EXTI_4_Count_NUM]
				&&_pin[EXTI_4_Count_NUM]==GPIO_Pin_4)
		{
			pin_last_vol[EXTI_4_Count_NUM]=RCS_GPIO_Read(_GPIO[EXTI_4_Count_NUM],_pin[EXTI_4_Count_NUM]);
			if(pin_last_vol[EXTI_4_Count_NUM]==1)	Current_Trigger=RCS_EXTI_RISE;
			else if(pin_last_vol[EXTI_4_Count_NUM]==0)	Current_Trigger=RCS_EXTI_FALL;
			
			EXTI_CB_Function[EXTI_4_Count_NUM](_GPIO[EXTI_4_Count_NUM],_pin[EXTI_4_Count_NUM],Current_Trigger);
		}
		EXTI_4_Count_NUM++;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(GPIO_Pin_4));
}

void Function_EXTI_9_5(void)
{
	RCS_EXTI_CURRENT_TRIGGER Current_Trigger; 
	int EXTI_9_5_Count_NUM=0; 
	uint16_t	target_pin;
	while(EXTI_CB_Function[EXTI_9_5_Count_NUM]!=NULL)
	{
		if(RCS_GPIO_Read(_GPIO[EXTI_9_5_Count_NUM],_pin[EXTI_9_5_Count_NUM])!=pin_last_vol[EXTI_9_5_Count_NUM]
				&&_pin[EXTI_9_5_Count_NUM]>=GPIO_Pin_5&&_pin[EXTI_9_5_Count_NUM]<=GPIO_Pin_9)
		{
			pin_last_vol[EXTI_9_5_Count_NUM]=RCS_GPIO_Read(_GPIO[EXTI_9_5_Count_NUM],_pin[EXTI_9_5_Count_NUM]);
			if(pin_last_vol[EXTI_9_5_Count_NUM]==1)	Current_Trigger=RCS_EXTI_RISE;
			else if(pin_last_vol[EXTI_9_5_Count_NUM]==0)	Current_Trigger=RCS_EXTI_FALL;
			target_pin=_pin[EXTI_9_5_Count_NUM];
			
			EXTI_CB_Function[EXTI_9_5_Count_NUM](_GPIO[EXTI_9_5_Count_NUM],_pin[EXTI_9_5_Count_NUM],Current_Trigger);
		}
		EXTI_9_5_Count_NUM++;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(target_pin));
}

void Function_EXTI15_10(void)
{
	RCS_EXTI_CURRENT_TRIGGER Current_Trigger; 
	int EXTI_15_10_Count_NUM=0; 
	uint16_t	target_pin;
	while(EXTI_CB_Function[EXTI_15_10_Count_NUM]!=NULL)
	{
		if(RCS_GPIO_Read(_GPIO[EXTI_15_10_Count_NUM],_pin[EXTI_15_10_Count_NUM])!=pin_last_vol[EXTI_15_10_Count_NUM]
				&&_pin[EXTI_15_10_Count_NUM]>=GPIO_Pin_10&&_pin[EXTI_15_10_Count_NUM]<=GPIO_Pin_15)
		{
			pin_last_vol[EXTI_15_10_Count_NUM]=RCS_GPIO_Read(_GPIO[EXTI_15_10_Count_NUM],_pin[EXTI_15_10_Count_NUM]);
			if(pin_last_vol[EXTI_15_10_Count_NUM]==1)	Current_Trigger=RCS_EXTI_RISE;
			else if(pin_last_vol[EXTI_15_10_Count_NUM]==0)	Current_Trigger=RCS_EXTI_FALL;
			target_pin=_pin[EXTI_15_10_Count_NUM];
			
			EXTI_CB_Function[EXTI_15_10_Count_NUM](_GPIO[EXTI_15_10_Count_NUM],_pin[EXTI_15_10_Count_NUM],Current_Trigger);
		}
		EXTI_15_10_Count_NUM++;
	}
	EXTI_ClearITPendingBit(GetRCS_EXTI_Line(target_pin));
}
