#include "RCS_DT35.h"

void DT35_Init(DT35_Handler_t* hdl,float min_mm,float max_mm,ADC_TypeDef* ADCx,uint8_t ADC_Channel_x,GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin_x)
{
	hdl->max_dist_mm=max_mm;
	hdl->min_dist_mm=min_mm;
	hdl->_ADCx=ADCx;
	hdl->_GPIOx=GPIOx;
	hdl->_pin=GPIO_Pin_x;
	hdl->channel=ADC_Channel_x;
	RCS_ADC_Init(ADCx,GPIOx,GPIO_Pin_x);
}

void DT35_Setup_WindowFilter(DT35_Handler_t* hdl,uint16_t window_size)
{
	WindowFloat_Init_Slide(hdl,window_size);
}

void DT35_Setup_KalmanFilter(DT35_Handler_t* hdl,float p,float q,float r,float gain)
{
	hdl->KF_Param.Last_P=p;
	hdl->KF_Param.Now_P=p;
	hdl->KF_Param.Q=q;
	hdl->KF_Param.R=r;
	hdl->KF_Param.K=gain;
	hdl->KF_Param.Predit=0;
}

uint16_t DT35_Get_Raw(DT35_Handler_t* hdl)
{
	return RCS_Get_ADC(hdl->_ADCx,hdl->channel);
}

float DT35_Get_dB(DT35_Handler_t* hdl)
{
	float reval;
	uint16_t adc_max=4095;
	reval=((float)(RCS_Get_ADC(hdl->_ADCx,hdl->channel)))/(float)adc_max;
	return reval;
}

float DT35_Get_MM(DT35_Handler_t* hdl)
{
	float reval;
	uint16_t adc_max=4095;
	reval=hdl->min_dist_mm+hdl->max_dist_mm*((float)(RCS_Get_ADC(hdl->_ADCx,hdl->channel)))/(float)adc_max;
	return reval;
}

float DT35_Get_Kalman(DT35_Handler_t* hdl)
{
	float input=(float)DT35_Get_MM(hdl);
  hdl->KF_Param.Now_P = hdl->KF_Param.Last_P + hdl->KF_Param.Q;
  hdl->KF_Param.K = hdl->KF_Param.Now_P / (hdl->KF_Param.Now_P + hdl->KF_Param.R);
	hdl->KF_Param.Predit = hdl->KF_Param.out;    
  hdl->KF_Param.out = hdl->KF_Param.out + hdl->KF_Param.K * (input - hdl->KF_Param.out); //????
  hdl->KF_Param.Last_P = (1-hdl->KF_Param.K) * hdl->KF_Param.Now_P;
	return hdl->KF_Param.out;
}
