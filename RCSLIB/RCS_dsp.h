/**
 @filename:RCS_dsp.h
 @brief:更快更准地进行数值计算
 @tips:如果定义了ARM_MATH_CM4，则自动选择DSP库实现浮点运算，反之则直接使用FPU进行浮点运算
 @tips:在.h中通过inline定义函数会有头文件重复包含的问题，因此请勿将该头文件放在rcs.h中，而是哪里要用放哪里
 @date:2023/10/30
 @author:陈煜楷
**/
#ifndef RCS_DSP_H_
#define RCS_DSP_H_
#include <math.h>
#include <stdint.h>
#include "cmsis_armclang.h"


/*****************************************************
        @addtogroup:常用常数定义
*****************************************************/
#define F32_PI          3.1415926535f 
#define F32_PI_TWICE    6.2831853072f 
#define F32_PI_HALF     1.5707963268f
#define F32_COS_45      0.707106781187f
#define F32_COS_30      0.866025403784f
#define F32_COS_60      0.5f

#define pi 				3.14159265357f        //π
#define DEG2RAD         1.74532925199433e-2f  //角度变换成弧度     
#define RAD2DEG         5.72957795130823e1f		//弧度变换成角度
#define FLOAT_ZERO      1e-10f                //浮点数相等的容差
#define G               9.8f                  //重力加速度
/*****************************************************
        @addtogroup:使用FPU进行简单计算
*****************************************************/
__INLINE float RCS_F32_Pow(float input)
{
	return (input*input);
}

__INLINE float RCS_F32_Sqrt(float input)
{
	float temp_output; 

	#ifdef ARM_MATH_CM4
	    arm_sqrt_f32(input,&temp_output);
	#else
	    temp_output=sqrtf(input);
	#endif
	
	return temp_output;
}

__INLINE float RCS_F32_Atan2(float y,float x)
{
    if ((y==0)&&(x==0))//避免进入hard_fault
    {
        return 0;
    }
    else
    {
	    float temp_output;
	    #ifdef ARM_MATH_CM4 
	        arm_atan2_f32(y,x,&temp_output);
	    #else 
	        temp_output=atan2f(y,x);
	    #endif
        return temp_output;
    }
}

__INLINE float RCS_F32_Sin(float input)
{
    #ifdef ARM_MATH_CM4 
	    return arm_sin_f32(input);
	#else 
	    return sinf(input);
	#endif
}

__INLINE float RCS_F32_Cos(float input)
{
    #ifdef ARM_MATH_CM4 
	    return arm_cos_f32(input);
	#else 
	    return cosf(input);
	#endif
}

__INLINE float RCS_F32_Modf_Fpart(float input)
{
    float temp;
    return modff(input,&temp);
}

__INLINE float RCS_F32_Trunc(float input)
{
    return truncf(input);
}

/*****************************************************
        @addtogroup:使用数值稳定的算法进行数值计算
*****************************************************/
/**
 @brief:sqrt(a*a+b*b)，但是不会因为a和b差距过大而损失精度
 @todo:暂时没有用cmsis_dsp库重写hypot的功能，仍然用的c标准库hypot
**/
__INLINE float RCS_F32_Hypot(float a,float b)
{
    #ifdef ARM_MATH_CM4
        return hypotf(a,b);
    #else
        return hypotf(a,b);
    #endif
}

/**
 @brief:x*y+z，但是只产生一次舍入误差而不是两次
 @todo:CM4貌似不支持三元运算指令，不清楚这个函数是否真的有效
**/
__INLINE float RCS_F32_Multi_Add(float x,float y,float z)
{
    return fmaf(x,y,z);
}

/**
 @brief: 返回num/den的整数商
**/
__INLINE int32_t RCS_F32_Rem_Quot(float num,float den)
{
    int32_t quot_output;
    remquof(num,den,&quot_output);
    return quot_output;
}

/**
 @brief: 返回num/den的浮点余数
**/
__INLINE float RCS_F32_Rem_Remainder(float num,float den)
{
    return remainderf(num,den);
}

/**
 @brief:将-∞~+∞的角度回环成[0,2*pi),并保证数值稳定
 @param:target_angle,弧度制角度
**/
__INLINE float Get_Loop_Angle_R(float target_angle)
{
	float temp_output=RCS_F32_Rem_Remainder(target_angle,F32_PI_TWICE);
	if (temp_output>=0) return temp_output;
	else                return temp_output+F32_PI_TWICE;
}
/*****************************************************
        @addtogroup:其他DSP功能
*****************************************************/

/**
 * @name:cumtrapz
 * @brief:梯形公式求解数值积分
*/
__INLINE void cumtrapz(double* input,double* output,uint16_t len)
{
    double sum=input[0];
    output[0]=0;

    for (int i=1;i<len;i++)
    {
        sum+=input[i];
        output[i]=sum-0.5*(input[0]+input[i]);
    }
}



#endif