#include "Cov_AcceLimit_Ctrl.h"

void CovDace_Limit_Init(float ac_max_1,float ac_max_2,float ac_max_3,float ac_max_4, 
	                      float dc_max_1,float dc_max_2,float dc_max_3,float dc_max_4,
												CovDace_Handler_t* hdl)
{
	hdl->Acce_Max[0]=ac_max_1;
	hdl->Acce_Max[1]=ac_max_2;
	hdl->Acce_Max[2]=ac_max_3;
	hdl->Acce_Max[3]=ac_max_4;
	hdl->Dcce_Max[0]=dc_max_1;
	hdl->Dcce_Max[1]=dc_max_2;
	hdl->Dcce_Max[2]=dc_max_3;
	hdl->Dcce_Max[3]=dc_max_4;
}

void CovDace_Limit_Set_DiffData(float ref_1,float ref_2,float ref_3,float ref_4,CovDace_Handler_t* hdl)
{
	hdl->Diff_Data[0]=ref_1;
	hdl->Diff_Data[1]=ref_2;
	hdl->Diff_Data[2]=ref_3;
	hdl->Diff_Data[3]=ref_4;
}

void CovDace_Limit_Get(float curr_1,float curr_2,float curr_3,float curr_4,CovDace_Handler_t* hdl)
{
	//input
	hdl->Dace_Data[0]=curr_1-hdl->Diff_Data[0];
	hdl->Dace_Data[1]=curr_2-hdl->Diff_Data[1];
	hdl->Dace_Data[2]=curr_3-hdl->Diff_Data[2];
	hdl->Dace_Data[3]=curr_4-hdl->Diff_Data[3];
	if (fabsf(curr_1)>=1e-4f) hdl->Curr_Data[0]=curr_1; else hdl->Curr_Data[0]=1e-4f;
	if (fabsf(curr_2)>=1e-4f) hdl->Curr_Data[1]=curr_2; else hdl->Curr_Data[1]=1e-4f;
	if (fabsf(curr_3)>=1e-4f) hdl->Curr_Data[2]=curr_3; else hdl->Curr_Data[2]=1e-4f;
	if (fabsf(curr_4)>=1e-4f) hdl->Curr_Data[3]=curr_4; else hdl->Curr_Data[3]=1e-4f;
	hdl->Dace_Overflow_Max=0;
	hdl->Dace_Overflow_Dir=0;
	
	//get the max overflowed id
	for (int i=0;i<4;i++)
	{
		//acce process
		if ((hdl->Dace_Data[i]>=0.0f)&&(hdl->Dace_Data[i]>=hdl->Acce_Max[i]))
		{
			if ((hdl->Dace_Data[i] - hdl->Acce_Max[i])>=hdl->Dace_Overflow_Max)
			{
				hdl->Dace_Overflow_Max=hdl->Dace_Data[i] - hdl->Acce_Max[i];
				hdl->Dace_Overflow_Ptr=i;
				hdl->Dace_Overflow_Dir=1;
			}
		}
		//dcce process
		else if ((hdl->Dace_Data[i]<=0.0f)&&(hdl->Dace_Data[i]<=hdl->Dcce_Max[i]))
		{
			if ((hdl->Dcce_Max[i] - hdl->Dace_Data[i])>=hdl->Dace_Overflow_Max)
			{
				hdl->Dace_Overflow_Max=hdl->Dcce_Max[i] - hdl->Dace_Data[i];
				hdl->Dace_Overflow_Ptr=i;
				hdl->Dace_Overflow_Dir=-1;
			}
		}
	}
	
	//get max flowed id`s output
	if (hdl->Dace_Overflow_Dir==1)
		hdl->Outp_Data[hdl->Dace_Overflow_Ptr]=hdl->Diff_Data[hdl->Dace_Overflow_Ptr] + hdl->Acce_Max[hdl->Dace_Overflow_Ptr];
	else if (hdl->Dace_Overflow_Dir==-1)
		hdl->Outp_Data[hdl->Dace_Overflow_Ptr]=hdl->Diff_Data[hdl->Dace_Overflow_Ptr] + hdl->Dcce_Max[hdl->Dace_Overflow_Ptr];
	else
		hdl->Outp_Data[hdl->Dace_Overflow_Ptr]=hdl->Curr_Data[hdl->Dace_Overflow_Ptr];
	
	//get other output
	hdl->Dace_Cov_Prop[0]=hdl->Curr_Data[0]/hdl->Curr_Data[hdl->Dace_Overflow_Ptr];
	hdl->Dace_Cov_Prop[1]=hdl->Curr_Data[1]/hdl->Curr_Data[hdl->Dace_Overflow_Ptr];
	hdl->Dace_Cov_Prop[2]=hdl->Curr_Data[2]/hdl->Curr_Data[hdl->Dace_Overflow_Ptr];
	hdl->Dace_Cov_Prop[3]=hdl->Curr_Data[3]/hdl->Curr_Data[hdl->Dace_Overflow_Ptr];
	
	hdl->Outp_Data[0]=hdl->Outp_Data[hdl->Dace_Overflow_Ptr]*hdl->Dace_Cov_Prop[0];
	hdl->Outp_Data[1]=hdl->Outp_Data[hdl->Dace_Overflow_Ptr]*hdl->Dace_Cov_Prop[1];
	hdl->Outp_Data[2]=hdl->Outp_Data[hdl->Dace_Overflow_Ptr]*hdl->Dace_Cov_Prop[2];
	hdl->Outp_Data[3]=hdl->Outp_Data[hdl->Dace_Overflow_Ptr]*hdl->Dace_Cov_Prop[3];
}


float SampleCtrl_Get(float input, StaySample_Handler_t* hdl)
{
	hdl->count++;
	if (hdl->count==hdl->rate) 
	{
		hdl->output=input;
		hdl->count=0;
	}
	return hdl->output;
}