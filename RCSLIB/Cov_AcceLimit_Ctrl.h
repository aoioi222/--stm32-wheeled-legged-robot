#ifndef COV_ACCELIMIT_CTRL_H_
#define COV_ACCELIMIT_CTRL_H_

#include "math.h"
#include "stdint.h"

typedef struct{
	float Acce_Max[4];
	float Dcce_Max[4];
	float Diff_Data[4];
	float Curr_Data[4];
	float Outp_Data[4];
	
	float Dace_Data[4];
	
	float   Dace_Overflow_Max;
	int     Dace_Overflow_Ptr;
	int     Dace_Overflow_Dir;
	float   Dace_Cov_Prop[4];
}CovDace_Handler_t;

typedef struct{
	uint8_t rate;
	uint8_t count;
	float   output;
}StaySample_Handler_t;


void CovDace_Limit_Init(float ac_max_1,float ac_max_2,float ac_max_3,float ac_max_4, 
	                      float dc_max_1,float dc_max_2,float dc_max_3,float dc_max_4,
												CovDace_Handler_t* hdl);
												
void CovDace_Limit_Set_DiffData(float ref_1,float ref_2,float ref_3,float ref_4,CovDace_Handler_t* hdl);
void CovDace_Limit_Get(float curr_1,float curr_2,float curr_3,float curr_4,CovDace_Handler_t* hdl);
							
float SampleCtrl_Get(float input, StaySample_Handler_t* hdl);												

#endif