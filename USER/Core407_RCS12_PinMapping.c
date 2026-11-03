/***************************正面************************************
*     US3  CAN1 CAN2       TIM_GROUP2/AIO_GROUP2    TIM_GROUP1/AIO_GROUP1 *
* TIM_GROUP_4/AIO_GROUP3                                 IO_GROUP_2       *

* US6                                                    ADC1/AIO_GROUP5  *

* US1                                                    IO_GROUP_1       *

* US2                                                    Cylinder_GROUP2  *

* TIM_GROUP3/AIO_GROUP4                                  Cylinder_GROUP1  *

*             SPI1             开关    电源插头                            *
***************************************************************************
                               开关    电源插头                            *
* US5                                                                      *

* IO_GROUP3                                                                *

* US4                                                     ADC2             *

* I2C                                                     IO_GROUP4        *

*                                                                          *

*             IO_GROUP5                                                    *
******************************背面******************************************/
#include "Core407_RCS12_PinMapping.h"

/* ============全局变量===================================================*/
RCS_PIN_USART    USART1_MAP,USART2_MAP,USART3_MAP,USART4_MAP,USART5_MAP,USART6_MAP;
RCS_PIN_CYLINDER CY1_MAP,CY2_MAP,CY3_MAP;
RCS_PIN_IO       IO1_MAP,IO2_MAP,IO3_MAP,IO4_MAP,AIO1_MAP,AIO2_MAP,AIO3_MAP,AIO4_MAP,AIO5_MAP;
RCS_PIN_IO       IO_DEBUG_MAP;
RCS_PIN_CAN      CAN1_MAP,CAN2_MAP;
RCS_PIN_TIM      TIMER1_MAP,TIMER2_MAP,TIMER3_MAP,TIMER4_MAP;

/* =============静态函数==================================================*/
static void RCS_TIM_PinMap_Init(void);
static void RCS_ADC_PinMap_Init(void);
static void RCS_IO_PinMap_Init(void);

/* =============函数实现==================================================*/

/**
 * @name:RCS_PinMap_Init/RCS_TIM_PinMap_Init/RCS_IO_PinMap_Init
 * @brief:定义主控板上的引脚
*/
void RCS_Core407_PinMap_Init(void)
{
    CAN1_MAP.CANx=CAN1;
    CAN1_MAP.GPIOx=GPIOA;
    CAN1_MAP.GPIO_Pin_Tx=GPIO_Pin_12;
    CAN1_MAP.GPIO_Pin_Rx=GPIO_Pin_11;

    CAN2_MAP.CANx=CAN2;
    CAN2_MAP.GPIOx=GPIOB;
    CAN2_MAP.GPIO_Pin_Tx=GPIO_Pin_13;
    CAN2_MAP.GPIO_Pin_Rx=GPIO_Pin_12;

    USART1_MAP.USARTx=USART1;
    USART1_MAP.GPIOx =GPIOA;
    USART1_MAP.GPIO_Pin_Tx=GPIO_Pin_9;
    USART1_MAP.GPIO_Pin_Rx=GPIO_Pin_10;

    USART2_MAP.USARTx=USART2;
    USART2_MAP.GPIOx =GPIOD;
    USART2_MAP.GPIO_Pin_Tx=GPIO_Pin_5;
    USART2_MAP.GPIO_Pin_Rx=GPIO_Pin_6;
	
	USART3_MAP.USARTx=USART3;
    USART3_MAP.GPIOx =GPIOD;
    USART3_MAP.GPIO_Pin_Tx=GPIO_Pin_8;
    USART3_MAP.GPIO_Pin_Rx=GPIO_Pin_9;

    USART4_MAP.USARTx=UART4;
    USART4_MAP.GPIOx =GPIOC;
    USART4_MAP.GPIO_Pin_Tx=GPIO_Pin_10;
    USART4_MAP.GPIO_Pin_Rx=GPIO_Pin_11;

    USART5_MAP.USARTx=UART5;
    USART5_MAP.GPIOx =GPIOD;
    USART5_MAP.GPIO_Pin_Tx=GPIO_Pin_12;
    USART5_MAP.GPIO_Pin_Rx=GPIO_Pin_2;

    USART6_MAP.USARTx=USART6;
    USART6_MAP.GPIOx =GPIOC;
    USART6_MAP.GPIO_Pin_Tx=GPIO_Pin_6;
    USART6_MAP.GPIO_Pin_Rx=GPIO_Pin_7;

    CY1_MAP.IO[0].GPIOx=GPIOE;
    CY1_MAP.IO[1].GPIOx=GPIOE;
    CY1_MAP.IO[2].GPIOx=GPIOE;
    CY1_MAP.IO[3].GPIOx=GPIOE;
    CY1_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_2;
    CY1_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_3;
    CY1_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_4;
    CY1_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_5;

    RCS_TIM_PinMap_Init();
    RCS_IO_PinMap_Init();
}
static void RCS_TIM_PinMap_Init(void)
{
    //第一路定时器(全是TIM1)
    TIMER1_MAP.IO[0].GPIOx=GPIOE;
    TIMER1_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_9;
    TIMER1_MAP.TIMx[0]=TIM1;
    TIMER1_MAP.Channel[0]=1;

    TIMER1_MAP.IO[1].GPIOx=GPIOE;
    TIMER1_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_11;
    TIMER1_MAP.TIMx[1]=TIM1;
    TIMER1_MAP.Channel[1]=2;

    TIMER1_MAP.IO[2].GPIOx=GPIOE;
    TIMER1_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_13;
    TIMER1_MAP.TIMx[2]=TIM1;
    TIMER1_MAP.Channel[2]=3;

    TIMER1_MAP.IO[3].GPIOx=GPIOE;
    TIMER1_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_14;
    TIMER1_MAP.TIMx[3]=TIM1;
    TIMER1_MAP.Channel[3]=4;

    //第二路定时器(TIM2+TIM12)
    TIMER2_MAP.IO[0].GPIOx=GPIOB;
    TIMER2_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_10;
    TIMER2_MAP.TIMx[0]=TIM2;
    TIMER2_MAP.Channel[0]=3;

    TIMER2_MAP.IO[1].GPIOx=GPIOB;
    TIMER2_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_11;
    TIMER2_MAP.TIMx[1]=TIM2;
    TIMER2_MAP.Channel[1]=4;

    TIMER2_MAP.IO[2].GPIOx=GPIOB;
    TIMER2_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_15;
    TIMER2_MAP.TIMx[2]=TIM12;
    TIMER2_MAP.Channel[2]=1;

    TIMER2_MAP.IO[3].GPIOx=GPIOB;
    TIMER2_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_14;
    TIMER2_MAP.TIMx[3]=TIM12;
    TIMER2_MAP.Channel[3]=2;

    //第四路定时器(全是TIM4)
    TIMER4_MAP.IO[0].GPIOx=GPIOD;
    TIMER4_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_13;
    TIMER4_MAP.TIMx[0]=TIM4;
    TIMER4_MAP.Channel[0]=2;

    TIMER4_MAP.IO[1].GPIOx=GPIOD;
    TIMER4_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_12;
    TIMER4_MAP.TIMx[1]=TIM4;
    TIMER4_MAP.Channel[1]=1;

    TIMER4_MAP.IO[2].GPIOx=GPIOD;
    TIMER4_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_15;
    TIMER4_MAP.TIMx[2]=TIM4;
    TIMER4_MAP.Channel[2]=4;

    TIMER4_MAP.IO[3].GPIOx=GPIOD;
    TIMER4_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_14;
    TIMER4_MAP.TIMx[3]=TIM4;
    TIMER4_MAP.Channel[3]=3;

    //第三路定时器(全是TIM4或者TIM4+TIM10+TIM11)
    TIMER3_MAP.IO[0].GPIOx=GPIOB;
    TIMER3_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_6;
    TIMER3_MAP.TIMx[0]=TIM4;
    TIMER3_MAP.Channel[0]=1;

    TIMER3_MAP.IO[1].GPIOx=GPIOB;
    TIMER3_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_7;
    TIMER3_MAP.TIMx[1]=TIM4;
    TIMER3_MAP.Channel[1]=2;

    TIMER3_MAP.IO[2].GPIOx=GPIOB;
    TIMER3_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_8;//TIM10_CH1或TIM4_CH3
    TIMER3_MAP.TIMx[2]=TIM10;
    TIMER3_MAP.Channel[2]=1;

    TIMER3_MAP.IO[3].GPIOx=GPIOB;
    TIMER3_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_9;//TIM11_CH1或TIM4_CH4
    TIMER3_MAP.TIMx[3]=TIM11;
    TIMER3_MAP.Channel[3]=1;  
}
static void RCS_IO_PinMap_Init(void)
{
    IO_DEBUG_MAP.IO[0].GPIOx=GPIOE;
    IO_DEBUG_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_0;
    IO_DEBUG_MAP.IO[1].GPIOx=GPIOE;
    IO_DEBUG_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_1;
    IO_DEBUG_MAP.IO[2].GPIOx=GPIOD;
    IO_DEBUG_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_3;
    IO_DEBUG_MAP.IO[3].GPIOx=GPIOD;
    IO_DEBUG_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_4;

    IO1_MAP.IO[0].GPIOx=GPIOE;
    IO1_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_2;
    IO1_MAP.IO[1].GPIOx=GPIOE;
    IO1_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_3;
    IO1_MAP.IO[2].GPIOx=GPIOE;
    IO1_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_4;
    IO1_MAP.IO[3].GPIOx=GPIOE;
    IO1_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_5;

    IO2_MAP.IO[0].GPIOx=GPIOC;
    IO2_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_4;
    IO2_MAP.IO[1].GPIOx=GPIOC;
    IO2_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_5;
    IO2_MAP.IO[2].GPIOx=GPIOB;
    IO2_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_0;
    IO2_MAP.IO[3].GPIOx=GPIOB;
    IO2_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_1;

    IO3_MAP.IO[0].GPIOx=GPIOD;
    IO3_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_7;
    IO3_MAP.IO[1].GPIOx=GPIOD;
    IO3_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_1;
    IO3_MAP.IO[2].GPIOx=GPIOD;
    IO3_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_0;
    IO3_MAP.IO[3].GPIOx=GPIOA;
    IO3_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_15;

    IO4_MAP.IO[0].GPIOx=GPIOE;
    IO4_MAP.IO[0].GPIO_Pin_x=GPIO_Pin_12;
    IO4_MAP.IO[1].GPIOx=GPIOE;
    IO4_MAP.IO[1].GPIO_Pin_x=GPIO_Pin_10;
    IO4_MAP.IO[2].GPIOx=GPIOE;
    IO4_MAP.IO[2].GPIO_Pin_x=GPIO_Pin_8;
    IO4_MAP.IO[3].GPIOx=GPIOE;
    IO4_MAP.IO[3].GPIO_Pin_x=GPIO_Pin_7;
}

/**
 * @name:RCS_Get_SYSCLK
 * @brief:获取MCU的时钟
*/
RCS_MCU_CLK RCS_Get_SYSCLK(void)
{
    RCS_MCU_CLK reval;
    RCC_GetClocksFreq(&reval);
    return reval;
}