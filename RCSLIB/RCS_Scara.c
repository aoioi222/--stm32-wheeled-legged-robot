/**
 * @name:RCS_Scara
 * @brief:存放二自由度机械臂解算算法
 * @usage:遵循先初始化再使用的逻辑，使用SCARA_PARAM创建多个机械臂实例，即可基于多机械臂的控制

 * @contribute:2024-1-13 CYK-dot 初次创建
 * -----------------------------------------------------
 * @changelog:2024-4-14 CYK-dot  完善接口
**/

/* 头文件 ======================================================*/
#include "RCS_Scara.h"
#include "RCS_dsp.h"

/* 私有全局变量 ================================================*/
volatile SCARA_AXIS output_scara;

/* 静态函数声明 ================================================*/


/* Scara接口函数定义 ===========================================*/

/**
 *@name: Scara_Param_Init
 *@brief:初始化一个SCARA机械臂实例
 *@param:Main_Len/End_Len            大小臂长度
 *@param:Main_Slowdown/End_Slowdown  大小臂减速比,右手坐标系为正减速比,反之为负
 *@param:Main_StartPos_Deg           大小臂的初始角度,按右手坐标系计算
 *@tips:电机角度应该在本函数之前就处理好,此函数只关注机械臂对外展现出的角度
**/
SCARA_PARAM Scara_Param_Init(SCARA_FLOAT_T Main_Len,         SCARA_FLOAT_T End_Len,
                             SCARA_FLOAT_T Main_Slowdown,    SCARA_FLOAT_T End_Slowdown,
                             SCARA_FLOAT_T Main_StartPos_Deg,SCARA_FLOAT_T End_StartPos_Deg)
{
	SCARA_PARAM reval;
	reval.Main_Len=Main_Len;
	reval.End_Len=End_Len;
	reval.Pinv_Epsilon=SCARA_PINV_COMMON;

	reval.Main_SlDn=Main_Slowdown;
	reval.End_SlDn=End_Slowdown;
	reval.Start_Pos.joint_angle_main=Main_StartPos_Deg*DEG2RAD;
	reval.Start_Pos.joint_angle_end=End_StartPos_Deg*DEG2RAD;
	return reval;
}


/**
 *@name: Scara_Spd_Ctrl
 *@brief:对SCARA机械臂进行速度控制
 *@param:spd_x/spd_y                         目标速度
 *@param:Main_MotorPos_Rad/End_MotorPos_Rad  大小臂电机输出轴的角度(弧度制)
 *@param:scara                               需要被控制的机械臂实例
 *@reval:scara两个电机的目标速度
**/
SCARA_AXIS Scara_Spd_Ctrl(SCARA_FLOAT_T spd_x,            SCARA_FLOAT_T spd_y,
                          SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,
                          SCARA_PARAM* scara)
{
		SCARA_AXIS Now_Pos_Scara;
		SCARA_AXIS Now_Pos_Motor;
		DESCARTES_AXIS Input_Spd;
		SCARA_AXIS     Output_Spd_s,Output_Spd_m;

		Now_Pos_Motor.joint_angle_main=Main_MotorPos_Rad;
		Now_Pos_Motor.joint_angle_end=End_MotorPos_Rad;
		Input_Spd.x=spd_x;
		Input_Spd.y=spd_y;

		//经过减速箱减速，获取机械臂角度
    Now_Pos_Scara=Motor_Trans_Scara_Pos(&Now_Pos_Motor,scara);

    //微分逆运动学，获取机械臂速度
    Input_Spd.x=spd_x*100.0;
    Input_Spd.y=spd_y*100.0;
    Output_Spd_s=Scara_Jcb_PInvKin(&Input_Spd,&Now_Pos_Scara,scara);

    //减速箱减速反过来，获取电机速度
    Output_Spd_m=Scara_Trans_Motor_Spd(&Output_Spd_s,scara);

    return Output_Spd_m;
}


/**
 *@name: Scara_Pos_Ctrl
 *@brief:对SCARA机械臂进行位置控制
 *@param:desc_pos_x/desc_pos_y               目标坐标
 *@param:Main_MotorPos_Rad/End_MotorPos_Rad  大小臂电机输出轴的角度(弧度制)
 *@param:scara                               需要被控制的机械臂实例
 *@reval:scara两个电机的位置
**/
SCARA_AXIS Scara_Pos_Ctrl(SCARA_FLOAT_T desc_pos_x,SCARA_FLOAT_T desc_pos_y,
                          SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,
                          SCARA_PARAM* scara)
{
		volatile SCARA_AXIS Now_Pos_Motor,Now_Pos_Scara;
		volatile SCARA_AXIS Output_Pos_Scara,Output_Pos_Motor;
		volatile DESCARTES_AXIS Input_Pos_Scara;
		//获取电机当前角度
		Now_Pos_Motor.joint_angle_main=Main_MotorPos_Rad;
		Now_Pos_Motor.joint_angle_end=End_MotorPos_Rad;

		//经过减速箱减速，获取机械臂当前角度
		Now_Pos_Scara=Motor_Trans_Scara_Pos(&Now_Pos_Motor,scara);

		//逆运动学获取机械臂目标角度
		Input_Pos_Scara.x=desc_pos_x;
		Input_Pos_Scara.y=desc_pos_y;
		Output_Pos_Scara=Scara_InvKin(&Input_Pos_Scara,&Now_Pos_Scara,scara);

		 //减速箱减速反过来，获取电机目标角度(rad)
		Output_Pos_Motor=Scara_Trans_Motor_Pos(&Output_Pos_Scara,scara);

		//debug test
		output_scara=Output_Pos_Motor;

		return Output_Pos_Motor;
}

/**
 *@name: Scara_Pos_Ctrl_Direct
 *@brief:直接控制scara电机的角度
 *@param:Main_Pos/End_Pos       目标角度(弧度制)
 *@param:scara                  需要被控制的机械臂实例
 *@reval:scara两个电机的位置
**/
SCARA_AXIS Scara_Pos_Ctrl_Direct(SCARA_FLOAT_T Main_Pos,SCARA_FLOAT_T End_Pos,SCARA_PARAM* scara)
{
	volatile SCARA_AXIS Target_Pos_Scara;
	volatile SCARA_AXIS Target_Pos_Motor;

	Target_Pos_Scara.joint_angle_main=Main_Pos;
	Target_Pos_Scara.joint_angle_end=End_Pos;
	Target_Pos_Motor=Scara_Trans_Motor_Pos(&Target_Pos_Scara,scara);
	output_scara=Target_Pos_Motor;
	return Target_Pos_Motor;
}

/**
 *@name: Scara_Get_Scara_Pos
 *@brief:直接获取scara机械臂的角度
 *@param:Main_MotorPos_Rad      电机角度(弧度制)
 *@param:scara                  机械臂实例
**/
SCARA_AXIS Scara_Get_Scara_Pos(SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,SCARA_PARAM* scara)
{
	SCARA_AXIS Now_Pos_Motor,Now_Pos_Scara;
	//获取电机当前角度
	Now_Pos_Motor.joint_angle_main=Main_MotorPos_Rad;
	Now_Pos_Motor.joint_angle_end=End_MotorPos_Rad;

	//经过减速箱减速，获取机械臂当前角度
	Now_Pos_Scara=Motor_Trans_Scara_Pos(&Now_Pos_Motor,scara);

	return Now_Pos_Scara;
}

/**
 *@name: Scara_Get_Desc_Pos
 *@brief:获取机械臂末端的笛卡尔坐标系位置
 *@param:Main_MotorPos_Rad      电机角度(弧度制)
 *@param:scara                  机械臂实例
**/
DESCARTES_AXIS Scara_Get_Desc_Pos(SCARA_FLOAT_T Main_MotorPos_Rad,SCARA_FLOAT_T End_MotorPos_Rad,SCARA_PARAM* scara)
{
	SCARA_AXIS Now_Pos_Motor,Now_Pos_Scara;
	DESCARTES_AXIS Now_Pos_Scara_Desc;
	//获取电机当前角度
	Now_Pos_Motor.joint_angle_main=Main_MotorPos_Rad;
	Now_Pos_Motor.joint_angle_end=End_MotorPos_Rad;

	//经过减速箱减速，获取机械臂当前角度
	Now_Pos_Scara=Motor_Trans_Scara_Pos(&Now_Pos_Motor,scara);

	Now_Pos_Scara_Desc=Scara_Kin(&Now_Pos_Scara,scara);
	return Now_Pos_Scara_Desc;
}




/* 静态函数定义 ======================================*/

/**
 * @name:Scara_Kin
 * @brief:正运动学解算获取当前的末端位置
 * @param:SCARA_AXIS* jonit_vector 关节的两个角度
 * @param:SCARA_PARAM* scara       机械臂的基本参数
 * @reval:末端相对机器人所在的笛卡尔坐标系
*/
DESCARTES_AXIS Scara_Kin(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara)
{
	//编译器优化变量
	DESCARTES_AXIS reval;
	SCARA_AXIS jonit_angle=*jonit_vector;
	//即x=l2*cos(a1+a2)+l1*cos(a1)
	reval.x= scara->End_Len   *SCARA_COS(jonit_angle.joint_angle_end+jonit_angle.joint_angle_main)
	        +scara->Main_Len  *SCARA_COS(jonit_angle.joint_angle_main);
	//即y=l2*sin(a1+a2)+l1*sin(a1)
	reval.y= scara->End_Len   *SCARA_SIN(jonit_angle.joint_angle_end+jonit_angle.joint_angle_main)
	        +scara->Main_Len  *SCARA_SIN(jonit_angle.joint_angle_main);

	return reval;
}
/**
 * @name:Scara_InvKin
 * @brief:逆运动学解算得到关节角度
 * @param:target_vector            目标位置相对于车体笛卡尔坐标系的坐标
 * @param:SCARA_AXIS* now_axis     当前的机械臂角度(弧度)
 * @param:SCARA_PARAM* scara       机械臂的基本参数
 * @reval:距离当前位置最近的一个逆运动学解
*/
SCARA_AXIS Scara_InvKin(DESCARTES_AXIS* target_vector,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{
	SCARA_AXIS reval,reval_1,reval_2;
	DESCARTES_AXIS target_desc=*(target_vector);

	SCARA_FLOAT_T e_end_1,e_main_1,e_end_2,e_main_2;
	SCARA_FLOAT_T tar1_end,tar1_main,tar2_end,tar2_main;

	//机械臂目标角度
	reval_1.joint_angle_end=acos(((target_desc.x)*(target_desc.x)+(target_desc.y)*(target_desc.y)-(scara->Main_Len)*(scara->Main_Len)-(scara->End_Len)*(scara->End_Len))/(2*scara->Main_Len*scara->End_Len));
	reval_1.joint_angle_main=atan2(target_desc.y,target_desc.x)-atan2(scara->End_Len*sin(reval_1.joint_angle_end),scara->Main_Len+scara->End_Len*cos(reval_1.joint_angle_end));

	reval_2.joint_angle_end=-reval_1.joint_angle_end;
	reval_2.joint_angle_main=atan2(target_desc.y,target_desc.x)-atan2(scara->End_Len*sin(reval_2.joint_angle_end),scara->Main_Len+scara->End_Len*cos(reval_2.joint_angle_end));

	//电机目标角度
	tar1_end=-(reval_1.joint_angle_end)*RAD2DEG *36.0;
	tar1_main=-(reval_1.joint_angle_main)*RAD2DEG *36.0/20.0*33.0;

	tar2_end=-(reval_2.joint_angle_end)*RAD2DEG *36.0;
	tar2_main=-(reval_2.joint_angle_main)*RAD2DEG *36.0/20.0*33.0;


	if((tar1_end-(now_axis->joint_angle_end))<=0)
	{
		e_end_1=now_axis->joint_angle_end-tar1_end;
	}
	else
	{
		e_end_1=tar1_end-now_axis->joint_angle_end;
	}

	if((tar1_main-(now_axis->joint_angle_main))<=0)
	{
		e_main_1=now_axis->joint_angle_main-tar1_main;
	}
	else
	{
		e_main_1=tar1_main-now_axis->joint_angle_main;
	}

	if((tar2_end-(now_axis->joint_angle_end))<=0)
	{
		e_end_2=now_axis->joint_angle_end-tar2_end;
	}
	else
	{
		e_end_2=tar2_end-now_axis->joint_angle_end;
	}

	if((tar2_main-(now_axis->joint_angle_main))<=0)
	{
		e_main_2=now_axis->joint_angle_main-tar2_main;
	}
	else
	{
		e_main_2=tar2_main-now_axis->joint_angle_main;
	}

	//法1
	if((e_end_1+e_main_1)<=(e_end_2+e_main_2))
	{
		reval=reval_1;
	}
	if((e_end_1+e_main_1)>(e_end_2+e_main_2))
	{
		reval=reval_2;
	}

	return reval_2;
}
/**
 * @name:Scara_Jcb_Kin
 * @brief:根据关节速度获取末端速度
 * @param:SCARA_AXIS* now_axis     当前的机械臂角度(弧度)
 * @param:SCARA_PARAM* scara       机械臂的基本参数
 * @todo
*/
DESCARTES_AXIS Scara_Jcb_Kin(SCARA_AXIS* jonit_vector,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{

}
/**
 * @name:Scara_Jcb_PInvKin
 * @brief:根据目标速度获取关节速度(雅各比伪逆矩阵+正则化矩阵法)
 * @param:DESCARTES_AXIS* target_speed 目标笛卡尔速度
 * @param:SCARA_AXIS* now_axis         当前的机械臂角度(弧度)
 * @param:SCARA_PARAM* scara       机械臂的基本参数
 * @attention:平均用时110us,拉慢2.2%的控制帧率
*/
SCARA_AXIS Scara_Jcb_PInvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{
	//局部变量
	SCARA_AXIS reval_spd;
	SCARA_FLOAT_T vt1_num;
	SCARA_FLOAT_T vt2_num;
	SCARA_FLOAT_T vtx_den;

	//预先计算
	SCARA_FLOAT_T lj         = scara->Pinv_Epsilon;
	SCARA_FLOAT_T L1_Sint1   = scara->Main_Len *SCARA_SIN(now_axis->joint_angle_main);
	SCARA_FLOAT_T L1_Cost1   = scara->Main_Len *SCARA_COS(now_axis->joint_angle_main);
	SCARA_FLOAT_T L2_Sin_t1t2= scara->End_Len  *SCARA_SIN(now_axis->joint_angle_end+now_axis->joint_angle_main);
	SCARA_FLOAT_T L2_Cos_t1t2= scara->End_Len  *SCARA_COS(now_axis->joint_angle_end+now_axis->joint_angle_main);
	SCARA_FLOAT_T L1L2_Sin_t12t2=scara->Main_Len*scara->End_Len*SCARA_SIN(now_axis->joint_angle_main+2*now_axis->joint_angle_end);
	SCARA_FLOAT_T L1L2_Cos_t12t2=scara->Main_Len*scara->End_Len*SCARA_COS(now_axis->joint_angle_main+2*now_axis->joint_angle_end);
	SCARA_FLOAT_T L2L2      = scara->End_Len*scara->End_Len;
	SCARA_FLOAT_T L1L1      = scara->Main_Len*scara->Main_Len;

	//vt1 num
	vt1_num= ( scara->End_Len*L1L2_Sin_t12t2 - L2L2*L1_Sint1 - 2*lj*L2_Sin_t1t2 - 2*lj*L1_Sint1)*target_speed->x
            +(-scara->End_Len*L1L2_Cos_t12t2 + L2L2*L1_Cost1 + 2*lj*L2_Cos_t1t2 + 2*lj*L1_Cost1)*target_speed->y;

	//vt2 num
	vt2_num=-target_speed->x*(2*lj*L2_Sin_t1t2 + L1L1*L2_Sin_t1t2 - scara->End_Len*L1L1*SCARA_SIN(now_axis->joint_angle_main - now_axis->joint_angle_end) - L2L2*L1_Sint1 +  scara->End_Len*L1L2_Sin_t12t2)
            +target_speed->y*(2*lj*L2_Cos_t1t2 + L1L1*L2_Cos_t1t2 - scara->End_Len*L1L1*SCARA_COS(now_axis->joint_angle_main - now_axis->joint_angle_end) - L2L2*L1_Cost1 +  scara->End_Len*L1L2_Cos_t12t2);

	//vt1&vt2 den
	vtx_den= (2*(L1L1*lj + 2*L2L2*lj + L1L1*L2L2 + lj*lj - L1L1*L2L2*SCARA_COS(now_axis->joint_angle_end)*SCARA_COS(now_axis->joint_angle_end) + 2*scara->End_Len*scara->Main_Len*lj*SCARA_COS(now_axis->joint_angle_end)));

	reval_spd.joint_angle_main=(vt1_num/vtx_den);
	reval_spd.joint_angle_end =(vt2_num/vtx_den);

	return reval_spd;
}
/**
 * @name:Scara_Jcb_InvKin
 * @brief:根据目标速度获取关节速度(雅各比逆矩阵法)
 * @tips:不建议使用，会出现奇异矩阵和无法求逆的问题
*/
SCARA_AXIS Scara_Jcb_InvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{
	SCARA_AXIS reval_spd;

	 if (fabs(now_axis->joint_angle_end) < FLOAT_ZERO)
	 {
	 	if (now_axis->joint_angle_end<0) now_axis->joint_angle_end-=FLOAT_ZERO;
	 	else                             now_axis->joint_angle_end+=FLOAT_ZERO;
	 }

	 reval_spd.joint_angle_main=  (target_speed->x*  (SCARA_COS(now_axis->joint_angle_main + now_axis->joint_angle_end)                  /SCARA_SIN(now_axis->joint_angle_end))
	                              +target_speed->y*  (SCARA_SIN(now_axis->joint_angle_main + now_axis->joint_angle_end)                  /SCARA_SIN(now_axis->joint_angle_end))
	 														 )
	                             /(scara->Main_Len);

	 reval_spd.joint_angle_end = -(scara->End_Len *target_speed->x* (SCARA_COS(now_axis->joint_angle_main + now_axis->joint_angle_end)/SCARA_SIN(now_axis->joint_angle_end))
	                              +scara->End_Len *target_speed->y* (SCARA_SIN(now_axis->joint_angle_main + now_axis->joint_angle_end)/SCARA_SIN(now_axis->joint_angle_end))
	                              +scara->Main_Len*target_speed->x* (SCARA_COS(now_axis->joint_angle_main)                            /SCARA_SIN(now_axis->joint_angle_end))
	                              +scara->Main_Len*target_speed->y* (SCARA_SIN(now_axis->joint_angle_main)                            /SCARA_SIN(now_axis->joint_angle_end))
	 															)
	                             /(scara->Main_Len*scara->End_Len);
}
/**
 * @name:Scara_Trans_Motor_Pos
 * @brief:根据关节当前的角度，获取当前的电机角度
 * @param:SCARA_AXIS* jonit_vector    关节的绝对角度(以x正轴方向为大臂0度角,以大臂向外方向为小臂0度角)
 * @param:SCARA_AXIS* start_jonit_pos 关节的绝对起始角度(以x正轴方向为大臂0度角,以大臂向外方向为小臂0度角)
 * @param:SCARA_PARAM* scara          机械臂的基本减速参数
 * @reval:SCARA_AXIS                  电机的真实角度
*/
SCARA_AXIS Scara_Trans_Motor_Pos(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=(jonit_vector->joint_angle_main - (scara->Start_Pos).joint_angle_main) / scara->Main_SlDn;
	reval.joint_angle_end =(jonit_vector->joint_angle_end  - (scara->Start_Pos).joint_angle_end)  / scara->End_SlDn;
	return reval;
}

/**
 * @name:Motor_Trans_Scara_Pos
 * @brief:根据电机当前的角度，获取当前的关节角度
 * @param:SCARA_AXIS* motor_vector    电机的真实角度
 * @param:SCARA_AXIS* start_jonit_pos 关节的绝对起始角度(以x正轴方向为大臂0度角,以大臂向外方向为小臂0度角)
 * @param:SCARA_PARAM* scara          机械臂的基本减速参数
 * @reval:SCARA_AXIS                  关节的绝对角度(以x正轴方向为大臂0度角,以大臂向外方向为小臂0度角)
*/
SCARA_AXIS Motor_Trans_Scara_Pos(SCARA_AXIS* motor_vector,SCARA_PARAM* scara)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=motor_vector->joint_angle_main * scara->Main_SlDn;
	reval.joint_angle_end=motor_vector->joint_angle_end * scara->End_SlDn;

	reval.joint_angle_main+=(scara->Start_Pos).joint_angle_main;
	reval.joint_angle_end+=(scara->Start_Pos).joint_angle_end;
	return reval;
}

/**
 * @name:Scara_Trans_Motor_Spd
 * @brief:根据关节当前的速度，获取当前的电机速度
*/
SCARA_AXIS Scara_Trans_Motor_Spd(SCARA_AXIS* jonit_vector,SCARA_PARAM* scara)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=jonit_vector->joint_angle_main / scara->Main_SlDn;
	reval.joint_angle_end =jonit_vector->joint_angle_end  / scara->End_SlDn;
	return reval;
}

/**
 * @name:Motor_Trans_Scara_Spd
 * @brief:根据电机当前的速度，获取当前的关节速度
 * @param:SCARA_AXIS* motor_vector 电机当前的速度
 * @param:SCARA_PARAM* scara       机械臂的基本减速参数
 * @reval:SCARA_AXIS               关节当前的速度
*/
SCARA_AXIS Motor_Trans_Scara_Spd(SCARA_AXIS* motor_vector,SCARA_PARAM* scara)
{
	SCARA_AXIS reval;
	reval.joint_angle_main=motor_vector->joint_angle_main * scara->Main_SlDn;
	reval.joint_angle_end=motor_vector->joint_angle_end * scara->End_SlDn;

	return reval;
}

/**
 * @name:Judge_Scara_Jcb_InvKin
 * @brief:判断机械臂在当前位姿的灵活性
 * @tips:对于二自由度机械臂,小臂完全伸展(角度=0)时无法提供x速度，此时失去一个自由度
 * @todo:使用二次型衡量运动能力
*/
SCARA_FLOAT_T Judge_Scara_Jcb_InvKin(DESCARTES_AXIS* target_speed,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{

}

/**
 * @name:Judge_Scara_Jcb_InvKin
 * @brief:判断逆运动学位置解是否存在
 * @tips:对于二自由度机械臂,无法移动到圆外的区域
*/
uint8_t Judge_Scara_InvKin(DESCARTES_AXIS* target_pos,SCARA_AXIS* now_axis,SCARA_PARAM* scara)
{
	if (((target_pos->x*target_pos->x+target_pos->y*target_pos->y)
	    -(scara->Main_Len+scara->End_Len)*(scara->Main_Len+scara->End_Len)
	    )
		>= -FLOAT_ZERO
	   )
		return 0;
	else
		return 1;

}



