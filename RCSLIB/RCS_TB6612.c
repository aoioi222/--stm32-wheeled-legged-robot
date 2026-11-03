#include "RCS_TB6612.h"




{
	RCS_GPIO_Output_Init(IN1_IOx_MAP.IO[IN1_Gh_channel].GPIOx,IN1_IOx_MAP.IO[IN1_Gh_channel].GPIO_Pin_x);
	RCS_GPIO_Output_Init(IN2_IOx_MAP.IO[IN2_Gh_channel].GPIOx,IN2_IOx_MAP.IO[IN2_Gh_channel].GPIO_Pin_x);
	PWMInit(TIMERx_MAP.TIMx,TIMERx_MAP.Channel[Gh_channel],TIMERx_MAP.IO[Gh_channel].GPIOx,TIMERx_MAP.IO[Gh_channel].GPIO_Pin_x,TIMER_Freq,_PWMHZ);
}

void BDC_Ctrl(