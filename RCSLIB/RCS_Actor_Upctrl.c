#include "RCS_Actor_Upctrl.h"


/*************************************************************************
 *                      @addtogroup:气缸控制                             *
*************************************************************************/
/**
 * @name:Cylinder_Init
 * @brief:配置主控板上的某个6pin气缸接口
 * @param:RCS_PIN_CYLINDER CYx_MAP 在RCS_Pin_Mapping.c中查看接入的是哪个口
*/
void Cylinder_Init(RCS_PIN_CYLINDER CYx_MAP)
{
    for(int i=0;i<4;i++)
        RCS_GPIO_Output_Init(CYx_MAP.IO[i].GPIOx,CYx_MAP.IO[i].GPIO_Pin_x);
}

/**
 * @name:Cylinder_SetBits
 * @brief:将气缸接口中的某一路置为高电平
 * @param:RCS_PIN_CYLINDER CYx_MAP 哪个气缸口
 * @param:uint8_t channel          哪一路,从左到右依次为0~3
*/
void Cylinder_SetBits(RCS_PIN_CYLINDER CYx_MAP,uint8_t channel)
{
    RCS_GPIO_Set(CYx_MAP.IO[channel].GPIOx,CYx_MAP.IO[channel].GPIO_Pin_x);
}

/**
 * @name:Cylinder_ResetBits
 * @brief:将气缸接口中的某一路置为低电平
 * @param:RCS_PIN_CYLINDER CYx_MAP 哪个气缸口
 * @param:uint8_t channel          哪一路,从左到右依次为0~3
*/
void Cylinder_ResetBits(RCS_PIN_CYLINDER CYx_MAP,uint8_t channel)
{
    RCS_GPIO_Reset(CYx_MAP.IO[channel].GPIOx,CYx_MAP.IO[channel].GPIO_Pin_x);
}



/*************************************************************************
 *                  @addtogroup:红外开关/普通IO控制                       *
*************************************************************************/
void IO_Init(RCS_PIN_IO IOx_MAP)
{
    for(int i=0;i<4;i++)
        RCS_GPIO_Output_Init(IOx_MAP.IO[i].GPIOx,IOx_MAP.IO[i].GPIO_Pin_x);
}
uint8_t IO_Read(RCS_PIN_IO IOx_MAP,uint8_t channel)
{
    return RCS_GPIO_Read(IOx_MAP.IO[channel].GPIOx,IOx_MAP.IO[channel].GPIO_Pin_x);
}
void IO_SetBits(RCS_PIN_IO IOx_MAP,uint8_t channel)
{
    RCS_GPIO_Set(IOx_MAP.IO[channel].GPIOx,IOx_MAP.IO[channel].GPIO_Pin_x);
}
void IO_ResetBits(RCS_PIN_IO IOx_MAP,uint8_t channel)
{
    RCS_GPIO_Reset(IOx_MAP.IO[channel].GPIOx,IOx_MAP.IO[channel].GPIO_Pin_x);
}


/*************************************************************************
 *                  @addtogroup:舵机控制/直流电机控制                      *
*************************************************************************/

/**
 * @name:Servo_Init
 * @brief:配置定时器的某个端口为PWM输出口
 * @param:RCS_PIN_TIM 主控板上的6pin定时器口，填入TIMER1_MAP~TIMER4_MAP
 * @param:channel     6pin定时器口共有4路定时器输出，从左到右分别是0123，直接填入0/1/2/3即可
 * @param:_PWMHZ      需要输出的PWM频率。注意，无法保证输出频率精确，只能近似
*/
void Servo_Init(RCS_PIN_TIM TIMERx_MAP,uint16_t Gh_channel,uint32_t _CLKHZ,uint32_t _PWMHZ,PWM_Device_t* device_name)
{
    //PWM初始化
    PWMInit(TIMERx_MAP.TIMx[Gh_channel],TIMERx_MAP.Channel[Gh_channel],
            TIMERx_MAP.IO[Gh_channel].GPIOx,TIMERx_MAP.IO[Gh_channel].GPIO_Pin_x,
            _CLKHZ,_PWMHZ);
    
    //设备初始化
    device_name->Device_Type         =Device_Type_Servo;
    device_name->Drv_TIMERx_MAP      =TIMERx_MAP;
    device_name->Drv_TIMERx_MAP_Chl  =Gh_channel;
    device_name->PWM_Percent         =0;
}

/**
 * @name:Servo_Output
 * @brief:让主控上的定时器口输出PWM波
 * @param:PWM_Device_t* 被初始化为舵机的PWM设备的指针
 * @param:_percent      浮点数占空比,不支持负数占空比
 * @todo:改为直接控制角度
*/
void Servo_Output(PWM_Device_t* Servo_Device_Name,float _percent)
{
    uint16_t gh_channel=Servo_Device_Name->Drv_TIMERx_MAP_Chl;

    PWMOutput(Servo_Device_Name->Drv_TIMERx_MAP.TIMx[gh_channel],
              Servo_Device_Name->Drv_TIMERx_MAP.Channel[gh_channel],
             _percent);

    Servo_Device_Name->PWM_Percent=_percent;
}

/**
 * @name:Servo_Init_Comm
 * @brief:将PWM设备配置为舵机
 * @param:Servo_Device_Name PWM设备句柄的指针
 * @param:TIMERx_MAP        使用哪一路TIM
 * @param:Gh_channel        使用本路TIM的第几个接口(0~3)
 * @param:Max_Angle         舵机的最大角度
 * @param:Min_BandWidth     在多少ms下,舵机前进到最小角度
 * @param:Max_BandWidth     在多少ms下,舵机前进到最大角度
 * @param:PWM_Freq          舵机的PWM工作频率
**/

void Servo_Init_Comm(PWM_Device_t* Servo_Device_Name,RCS_PIN_TIM TIMERx_MAP,uint16_t Gh_channel,
                     float Max_Angle,float Min_BandWidth,float Max_BandWidth,uint16_t PWM_Freq)
{
    //配置PWM设备
    Servo_Init(TIMERx_MAP,Gh_channel,84000,PWM_Freq,Servo_Device_Name);
    //配置舵机
    Servo_Device_Name->Servo_Max_Angle =Max_Angle;
    Servo_Device_Name->Servo_Min_BWidth=Min_BandWidth;
    Servo_Device_Name->Servo_Max_BWidth=Max_BandWidth;
    Servo_Device_Name->Servo_Pwm_BWidth=1000.0f/(float)PWM_Freq;
}
/**
 * @name:Servo_Init_MG996r
 * @brief:将PWM设备配置为MG996r舵机
 * @tips:MG996R支持的有效带宽0.5ms~2.5ms，PWM频率50Hz
**/
void Servo_Init_MG996r(PWM_Device_t* Servo_Device_Name,RCS_PIN_TIM TIMERx_MAP,uint16_t Gh_channel,float Max_Angle,float Origin_Angle)
{
	Servo_Init_Comm(Servo_Device_Name,TIMERx_MAP,Gh_channel,Max_Angle,0.5f,2.5f,50);
	Servo_Device_Name->Servo_angle = Origin_Angle;
}
float   fask_angle;
int eva_angle(PWM_Device_t* Servo_Device_Name, float target_angle)
{
    float speed_up = 0.01f; // 加速度因子
    float current_angle = Servo_Device_Name->Servo_angle;
					{
            float angle_difference = target_angle - current_angle;
            float step = angle_difference * speed_up;
            current_angle += step;
       
            Servo_Ctrl(Servo_Device_Name, current_angle); // 控制舵机转到新的角度

            if (fabsf(target_angle - current_angle) <= 0.1) // 检查是否接近目标角度
            {
							
                Servo_Device_Name->reached_target = 1;
               
            }
         }
	
    return Servo_Device_Name->reached_target; // 返回是否达到目标角度
}


void Servo_Ctrl(PWM_Device_t* Servo_Device_Name,float _angle)
{
		//variables
    float BandWidth;
		//excute to hardware
    if (_angle >=0.0f && _angle<=Servo_Device_Name->Servo_Max_Angle)
    {
			
        BandWidth=Servo_Device_Name->Servo_Min_BWidth +
                 (_angle-Servo_Device_Name->Servo_Min_BWidth)*(Servo_Device_Name->Servo_Max_BWidth-Servo_Device_Name->Servo_Min_BWidth)
                /(Servo_Device_Name->Servo_Max_Angle);
       
        Servo_Output(Servo_Device_Name,BandWidth/Servo_Device_Name->Servo_Pwm_BWidth);
    }
   
		//update log
		Servo_Device_Name->Servo_Current_BWidth=BandWidth;
		Servo_Device_Name->Servo_angle=_angle;
		
}
/**
 * @name:Bdc_Init
 * @brief:将一路PWM与两路IO绑定到TB6612逻辑的直流PWM控制设备上
 * @param:RCS_PIN_TIM TIMERx_MAP,uint8_t Gh_channel 指定主控板上的一个定时器gh1.25接口和通道作为驱动输出
 * @param:RCS_PIN_IO IN1_IOx_MAP,uint8_t IN1_Gh_channel 指定主控板上的一个IOgh1.25接口和通道作为IN1
 * @param:RCS_PIN_IO IN2_IOx_MAP,uint8_t IN2_Gh_channel 指定主控板上的一个IOgh1.25接口和通道作为IN2
 * @param:PWM_Device_t* device_name 要被初始化为直流电机的PWM设备的指针
*/
void Bdc_Init(RCS_PIN_TIM TIMERx_MAP,uint8_t Gh_channel,uint32_t _CLKHZ,uint32_t _PWMHZ,
              RCS_PIN_IO IN1_IOx_MAP,uint8_t IN1_Gh_channel,
              RCS_PIN_IO IN2_IOx_MAP,uint8_t IN2_Gh_channel,
              PWM_Device_t* device_name)
{
    //PWM初始化
    PWMInit(TIMERx_MAP.TIMx[Gh_channel],TIMERx_MAP.Channel[Gh_channel],
            TIMERx_MAP.IO[Gh_channel].GPIOx,TIMERx_MAP.IO[Gh_channel].GPIO_Pin_x,
            _CLKHZ,_PWMHZ);
    //IO初始化
    RCS_GPIO_Output_Init(IN1_IOx_MAP.IO[IN1_Gh_channel].GPIOx,IN1_IOx_MAP.IO[IN1_Gh_channel].GPIO_Pin_x);
    RCS_GPIO_Output_Init(IN2_IOx_MAP.IO[IN2_Gh_channel].GPIOx,IN2_IOx_MAP.IO[IN2_Gh_channel].GPIO_Pin_x);

    //设备初始化
    device_name->Device_Type         =Device_Type_Bdc;
    device_name->Drv_TIMERx_MAP      =TIMERx_MAP;
    device_name->Drv_TIMERx_MAP_Chl  =Gh_channel;
    device_name->IN1_IOx_MAP         =IN1_IOx_MAP;
    device_name->IN1_IOx_MAP_Chl     =IN1_Gh_channel;
    device_name->IN2_IOx_MAP         =IN2_IOx_MAP;
    device_name->IN2_IOx_MAP_Chl     =IN2_Gh_channel;
    device_name->PWM_Percent         =0;
}

/**
 * @name:Bdc_Ctrl
 * @brief:控制直流电机的功率
 * @param:PWM_Device_t* Bdc_Device_Name 被初始化为直流电机的PWM设备的指针
 * @param:float real_percent -1~+1的占空比，负数即为反转
*/
void Bdc_Ctrl(PWM_Device_t* Bdc_Device_Name,float real_percent)
{
    //管脚获取
    RCS_MAP_IO   IN1_IO=Bdc_Device_Name->IN1_IOx_MAP.IO[Bdc_Device_Name->IN1_IOx_MAP_Chl]; 
    RCS_MAP_IO   IN2_IO=Bdc_Device_Name->IN2_IOx_MAP.IO[Bdc_Device_Name->IN2_IOx_MAP_Chl]; 
    TIM_TypeDef* Drv_TIMx=Bdc_Device_Name->Drv_TIMERx_MAP.TIMx[Bdc_Device_Name->Drv_TIMERx_MAP_Chl];
    uint8_t      Drv_TIMx_Chl=Bdc_Device_Name->Drv_TIMERx_MAP.Channel[Bdc_Device_Name->Drv_TIMERx_MAP_Chl];
    //输出
    if (real_percent <0)
    {
				if(real_percent>=-1)
					Bdc_Device_Name->PWM_Percent=-real_percent;
				else
					Bdc_Device_Name->PWM_Percent=-0.96;
				
        RCS_GPIO_Reset(IN1_IO.GPIOx,IN1_IO.GPIO_Pin_x);
        RCS_GPIO_Set(IN2_IO.GPIOx,IN2_IO.GPIO_Pin_x);
        PWMOutput(Drv_TIMx,Drv_TIMx_Chl,Bdc_Device_Name->PWM_Percent);
    }
    else if (real_percent >0)
    {
				if(real_percent<=1)
					Bdc_Device_Name->PWM_Percent=-real_percent;
				else
					Bdc_Device_Name->PWM_Percent=0.96;
       
        RCS_GPIO_Set(IN1_IO.GPIOx,IN1_IO.GPIO_Pin_x);
        RCS_GPIO_Reset(IN2_IO.GPIOx,IN2_IO.GPIO_Pin_x);
        PWMOutput(Drv_TIMx,Drv_TIMx_Chl,Bdc_Device_Name->PWM_Percent);
    }
    else
    {
        Bdc_Device_Name->PWM_Percent=0;
        RCS_GPIO_Reset(IN1_IO.GPIOx,IN1_IO.GPIO_Pin_x);
        RCS_GPIO_Reset(IN2_IO.GPIOx,IN2_IO.GPIO_Pin_x);
        PWMOutput(Drv_TIMx,Drv_TIMx_Chl,0);
    }
}

/* =============静态函数====================================================*/
