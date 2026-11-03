#ifndef  _MYLIB_H_
#define  _MYLIB_H_

/********************选择*******************/



/******************参数定义*****************/

#define pi				3.14159265357			//π
#define DEG2RAD			1.74532925199433e-2f	//角度变换成弧度     
#define RAD2DEG			5.72957795130823e1f		//弧度变换成角度

/******************函数定义*****************/
#define setbit(x,y)		x |=  (1<<y) 			//将X的第Y位置1
#define resetbit(x,y)     x &= ~(1<<y) 			//将X的第Y位清0

/************定时器以及中断函数定义***************/




/***************中断优先级定义**************/
#define UltrasonicWave1_TIM_Priority	0x13
#define UltrasonicWave1_Exit_Priority	0x14
#define UltrasonicWave2_TIM_Priority	0x11
#define UltrasonicWave2_Exit_Priority	0x12
#define Color1_Exit_Prioty				0x17
#define L_Trail_GPIO_Exit_Priority		0x15
#define KEY_WK_Priority 				0x40
#define KEY_K1_Priority 				0x41
#define KEY_K0_Priority 				0x42
#define MotorSpeed_Read_TIM_Priority	0x1B
#define MotorSpeed_Read_TIM_Priority_	0x05
#define USART1_Priority					0x22
#define Zservo_USART_Priority			0x23
#define Hi229_USART_Priority			0x04
#define EncoderLF_Priority             	0x16
#define EncoderLF_Confirm_Priority		0x00
#define EncoderLB_Priority             	0x18
#define EncoderLB_Confirm_Priority		0x01
#define EncoderRB_Priority            	0x19
#define EncoderRB_Confirm_Priority		0x02
#define EncoderRF_Priority             	0x1A
#define EncoderRF_Confirm_Priority		0x03

/******************管脚定义*****************/
#define KEY_WK_UP_GPIO					GPIOA
#define KEY_WK_UP_GPIO_Pin				GPIO_Pin_0
#define KEY_K1_GPIO						GPIOE
#define KEY_K1_GPIO_Pin					GPIO_Pin_3
#define KEY_K0_GPIO						GPIOE
#define KEY_K0_GPIO_Pin					GPIO_Pin_4
					
#define LED_D1_GPIO						GPIOF
#define LED_D1_GPIO_Pin					GPIO_Pin_9
#define LED_D2_GPIO						GPIOF
#define LED_D2_GPIO_Pin					GPIO_Pin_10

#define MotorRB_TIM						TIM13
#define MotorRB_CH						1
#define MotorRB_GPIO					GPIOF
#define MotorRB_GPIO_Pin				GPIO_Pin_8
#define MotorRB_CLKHZ					840000
#define MotorRB_PWMHZ					10000
#define MotorRB_IN1_GPIO 				GPIOF
#define MotorRB_IN1_GPIO_Pin			GPIO_Pin_14
#define MotorRB_IN2_GPIO 				GPIOF
#define MotorRB_IN2_GPIO_Pin			GPIO_Pin_12
#define MotorRF_TIM						TIM10
#define MotorRF_CH						1
#define MotorRF_GPIO					GPIOF
#define MotorRF_GPIO_Pin				GPIO_Pin_6
#define MotorRF_CLKHZ					840000
#define MotorRF_PWMHZ					10000
#define MotorRF_IN1_GPIO 				GPIOG
#define MotorRF_IN1_GPIO_Pin			GPIO_Pin_0
#define MotorRF_IN2_GPIO 				GPIOF
#define MotorRF_IN2_GPIO_Pin			GPIO_Pin_15
#define MotorLB_TIM						TIM14
#define MotorLB_CH						1
#define MotorLB_GPIO					GPIOF
#define MotorLB_GPIO_Pin				GPIO_Pin_9
#define MotorLB_CLKHZ					840000
#define MotorLB_PWMHZ					10000
#define MotorLB_IN1_GPIO 				GPIOF
#define MotorLB_IN1_GPIO_Pin			GPIO_Pin_13
#define MotorLB_IN2_GPIO 				GPIOF
#define MotorLB_IN2_GPIO_Pin			GPIO_Pin_11
#define MotorLF_TIM						TIM11
#define MotorLF_CH						1
#define MotorLF_GPIO					GPIOF
#define MotorLF_GPIO_Pin				GPIO_Pin_7
#define MotorLF_CLKHZ					840000
#define MotorLF_PWMHZ					10000
#define MotorLF_IN1_GPIO 				GPIOE
#define MotorLF_IN1_GPIO_Pin			GPIO_Pin_7
#define MotorLF_IN2_GPIO 				GPIOG
#define MotorLF_IN2_GPIO_Pin			GPIO_Pin_1

#define ENCODERLF_TIM					TIM8
#define ENCODERLF_GPIO 					GPIOC
#define ENCODERLF_GPIO_Pin_A			GPIO_Pin_6
#define ENCODERLF_GPIO_Pin_B			GPIO_Pin_7
#define ENCODERLF_GPIO_Pin_Test			GPIO_Pin_15
#define ENCODERLB_TIM					TIM4
#define ENCODERLB_GPIO 					GPIOB
#define ENCODERLB_GPIO_Pin_A			GPIO_Pin_6
#define ENCODERLB_GPIO_Pin_B			GPIO_Pin_7
#define ENCODERLB_GPIO_Pin_Test			GPIO_Pin_15
#define ENCODERRB_TIM					TIM3
#define ENCODERRB_GPIO 					GPIOA
#define ENCODERRB_GPIO_Pin_A			GPIO_Pin_6
#define ENCODERRB_GPIO_Pin_B			GPIO_Pin_7
#define ENCODERRB_GPIO_Pin_Test			GPIO_Pin_15
#define ENCODERRF_TIM					TIM1
#define ENCODERRF_GPIO 					GPIOE
#define ENCODERRF_GPIO_Pin_A			GPIO_Pin_9
#define ENCODERRF_GPIO_Pin_B			GPIO_Pin_11
#define ENCODERRF_GPIO_Pin_Test			GPIO_Pin_15

#define MotorSpeed_TIM 					TIM5
#define MotorSpeed_TIM_Period			120
#define MotorSpeed_TIM_Div				60000

#define MotorSpeed_TIM_ 				TIM6
#define MotorSpeed_TIM_Period_			60000
#define MotorSpeed_TIM_Div_				60000
	
#define Servo1_TIM						TIM9
#define Servo1_CH						1
#define Servo1_GPIO						GPIOE
#define Servo1_GPIO_Pin					GPIO_Pin_5
#define Servo1_CLKHZ					840000
#define Servo1_PWMHZ					50
#define Servo2_TIM						TIM9
#define Servo2_CH						2
#define Servo2_GPIO						GPIOE
#define Servo2_GPIO_Pin					GPIO_Pin_6
#define Servo2_CLKHZ					840000
#define Servo2_PWMHZ					50

#define Zservo_USART 					USART3
#define Zservo_USART_GPIO 				GPIOB
#define Zservo_USART_GPIO_Pin_TX		GPIO_Pin_10
#define Zservo_USART_GPIO_Pin_RX		GPIO_Pin_11
#define Zservo_USART_BaudRate			115200

#define Hi229_USART 					USART2
#define Hi229_USART_GPIO 				GPIOA
#define Hi229_USART_GPIO_Pin_TX			GPIO_Pin_2
#define Hi229_USART_GPIO_Pin_RX			GPIO_Pin_3
#define Hi229_USART_BaudRate			115200

#define USART1_GPIO						GPIOA
#define USART1_GPIO_Pin_TX				GPIO_Pin_9
#define USART1_GPIO_Pin_RX				GPIO_Pin_10
#define USART1_BaudRate					9600

#define UltrasonicWave1_GPIO 			GPIOG
#define UltrasonicWave1_GPIO_Pin_Echo	GPIO_Pin_12
#define UltrasonicWave1_GPIO_Pin_Trig	GPIO_Pin_11
#define UltrasonicWave1_TIM				TIM2
#define UltrasonicWave1_TIM_Period		65536
#define UltrasonicWave1_TIM_Div			1000000
#define UltrasonicWave2_GPIO 			GPIOG
#define UltrasonicWave2_GPIO_Pin_Echo	GPIO_Pin_10
#define UltrasonicWave2_GPIO_Pin_Trig	GPIO_Pin_9
#define UltrasonicWave2_TIM				TIM7
#define UltrasonicWave2_TIM_Period		65536
#define UltrasonicWave2_TIM_Div			1000000

#define Color1_S0_GPIO					GPIOF
#define Color1_S1_GPIO					GPIOF
#define Color1_S2_GPIO					GPIOF
#define Color1_S3_GPIO					GPIOF
#define Color1_OUT_GPIO					GPIOF
#define Color1_LED_GPIO					GPIOF
#define Color1_S0_GPIO_Pin				GPIO_Pin_0
#define Color1_S1_GPIO_Pin				GPIO_Pin_1
#define Color1_S2_GPIO_Pin				GPIO_Pin_2
#define Color1_S3_GPIO_Pin				GPIO_Pin_3
#define Color1_OUT_GPIO_Pin				GPIO_Pin_4
#define Color1_LED_GPIO_Pin				GPIO_Pin_5

#define F_Trail_L1_GPIO 				GPIOC
#define F_Trail_L1_GPIO_Pin				GPIO_Pin_0
#define F_Trail_L2_GPIO 				GPIOC
#define F_Trail_L2_GPIO_Pin				GPIO_Pin_2
#define F_Trail_M_GPIO 					GPIOC
#define F_Trail_M_GPIO_Pin				GPIO_Pin_1
#define F_Trail_R2_GPIO 				GPIOC
#define F_Trail_R2_GPIO_Pin				GPIO_Pin_3
#define F_Trail_R1_GPIO 				GPIOA
#define F_Trail_R1_GPIO_Pin				GPIO_Pin_4
#define L_Trail_GPIO 					GPIOA
#define L_Trail_GPIO_Pin				GPIO_Pin_5
#define R_Trail_GPIO 					GPIOC
#define R_Trail_GPIO_Pin				GPIO_Pin_4


/******************库包含*******************/

#include "stm32f4xx.h"
#include "ucos_ii.h"
#include "app_cfg.h"
#include "delay.h"
#include "bsp.h"
#include "cpu.h"

#include "My_Types.h"
#include "My_GPIO.h"
#include "My_OLED_ASCII.h"
#include "My_OLED.h"
#include "My_ADC.h"
#include "My_Exti.h"
#include "My_Usart.h"
#include "My_TimerInterrupt.h"
#include "My_Pwm.h"
#include "My_Encoder.h"

#include "led.h"
#include "key.h"
#include "MainTask.h"
#include "motor.h"
#include "servo.h"
#include "zservo.h"
#include "PID.h"
#include "usart.h"
#include "ultrasonic.h"
#include "color.h"
#include "trail.h"
#include "hi229.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#endif //_MYLIB_H_
