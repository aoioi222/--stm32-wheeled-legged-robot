
///* ====================头文件=====================================*/
//#include "Up_Ctrl.h"
//#include "RCS_Motor_Reposition.h"
//#include "R2_CombCtrl.h"
//#include "RCS_Dribble.h"
//#include "RCS_AGV_BaseMove.h"

//static uint8_t dribble_state = 0;
//static uint32_t dribble_timer = 0;
// uint8_t key_cylinder_state = 0;

//float speed_x = 0;
//float speed_y = 0;
//float speed_z = 0;

//RCS_Reset_Err_t test1,test2;
//float center_pos0=0.0f,center_pos1=0.0f;
//float target_pos0=0.0f,target_pos1=0.0f;
//extern Motor_Ctrl_Node agv_drv[4], crazy_fly[4], shoot[2];
//extern int knob_l,knob_r;



//void Myinterrupt()
//{
//	
//}



//void Remote_control_car()
//{
//	if(key_flag[2]==0)
//	{
//		
////		MotorNode_Update_Spd(500,&crazy_fly[0]);
////		MotorNode_Update_Spd(500,&crazy_fly[1]);
//		uint16_t Re_set_state = 0;
//		//	thetas=Get_Gyro_Z();	
//		if(abs(rocker_lx-125)<2) rocker_lx=126;//键位归中
//		if(abs(rocker_ly-125)<2) rocker_ly=126;
//		if(abs(rocker_rx-126)<2) rocker_rx=126;
//		if(abs(rocker_ry-126)<2) rocker_ry=126;		
//		
//		if(key_flag[3] == 0)
//		{
//			speed_y=((float)(rocker_ly-126))*2;
//			speed_x=((float)(rocker_lx-126))*2;
//			speed_z=((float)(rocker_rx-126))*2;
//		}
//		else 
//		{
//			speed_y=((float)(rocker_ly-126))*4;
//			speed_x=((float)(rocker_lx-126))*4;
//			speed_z=((float)(rocker_rx-126))*4;
//		}

//	
//	
//		if(speed_x == 0 && speed_y == 0 && speed_z == 0)
//		{
////			Motor_Send(0, 0, 0, 0);
//			MotorNode_Update_Spd(0,&agv_drv[0]);
//			MotorNode_Update_Spd(0,&agv_drv[1]);
//		}
//		else
//		{
//			motor_reset_angle_update(GPIO_Pin_13,1,0,rcsSrv_Rep_RealAngle0);
//			motor_reset_angle_update(GPIO_Pin_6,2,0,rcsSrv_Rep_RealAngle1);
//			
//			AGV_Re_Center_Move_v3_3(speed_x, speed_y, -speed_z);
//			//HAL_Chassis_Ctrl(speed_x, speed_y, -speed_z);
//		}


//		if(stop_key == 0)
//		{
//			
//			if(Re_set_state == 0)
//			{
//				test1=motor_reset_ir(&rcsMotor_ctrlSpd_0,&rcsSrv_getPos_fptr_0,GPIOD,GPIO_Pin_13,&center_pos0,1, 0);
//				test2=motor_reset_ir(&rcsMotor_ctrlSpd_1,&rcsSrv_getPos_fptr_1,GPIOB,GPIO_Pin_6,&center_pos1,2, 0);
//				if(((test1 == RCS_Reset_OK) && (test2 == RCS_Reset_OK)))
//				{
//					Re_set_state = 1;
//				}
//			}
//			if(Re_set_state == 1)
//			{
//				rcsAgv0_Update_Angle_0(target_pos0);
//				rcsAgv0_Update_Angle_1(target_pos1);
//				if(Count_Delay(300,1))
//				{
//					Re_set_state = 2;
//				}
//			}
//		}
//	}
//	else
//	{
//		Motor_Send(0, 0, 0, 0);
//		MotorNode_Update_Spd(0,&agv_drv[0]);
//		MotorNode_Update_Spd(0,&agv_drv[1]);
////		MotorNode_Update_Spd(0,&crazy_fly[0]);
////		MotorNode_Update_Spd(0,&crazy_fly[1]);
//		
//	}
//}



//uint16_t state = 0;


////气缸+光电门
//void cylinder_init(void)
// {

//      RCS_GPIO_Output_Init(GPIOE,GPIO_Pin_2);      //夹爪气缸GPIO初始化
//      RCS_GPIO_Output_Init(GPIOE,GPIO_Pin_3);
//      RCS_GPIO_Output_Init(GPIOE,GPIO_Pin_4);
//      RCS_GPIO_Output_Init(GPIOE,GPIO_Pin_5);
//	 
//	  //  RCS_GPIO_Input_Init(GPIOB,GPIO_Pin_0);
//	    RCS_GPIO_Input_Init(GPIOD,GPIO_Pin_15);     //光电门GPIO初始化
//	 
//			RCS_GPIO_Input_Init(GPIOD,GPIO_Pin_12);     //滑轨光电门初始化
//	 
//	    RCS_GPIO_Output_Init(GPIOE,GPIO_Pin_6);     //跳跃气缸GPIO初始化
//	    RCS_GPIO_Output_Init(GPIOC,GPIO_Pin_13);
//      RCS_GPIO_Output_Init(GPIOC,GPIO_Pin_14);
//	    RCS_GPIO_Output_Init(GPIOC,GPIO_Pin_15);
// }
// 


////夹爪气缸
//void cylinder_ctrl(uint8_t cylinder_state)
//{
//    //UpCtrl_Angle(angle_test);
//    if(cylinder_state == 0)
//    {
//    //RCS_GPIO_Set(GPIOE,GPIO_Pin_2);
//    RCS_GPIO_Set(GPIOE,GPIO_Pin_3);

//    }
//    if(cylinder_state == 1)
//    {
//    //RCS_GPIO_Reset(GPIOE,GPIO_Pin_2);
//    RCS_GPIO_Reset(GPIOE,GPIO_Pin_3);
//    }
//}

////推动运球气缸
//void push_ctrl(uint8_t cylinder_state)
//{
//    //UpCtrl_Angle(angle_test);
//    if(cylinder_state == 1)
//    {
//    RCS_GPIO_Set(GPIOE,GPIO_Pin_2);
//    //RCS_GPIO_Set(GPIOE,GPIO_Pin_4);

//    }
//    if(cylinder_state == 0)
//			
//		
//    {
//    RCS_GPIO_Reset(GPIOE,GPIO_Pin_2);
//    //RCS_GPIO_Reset(GPIOE,GPIO_Pin_4);

//    }
//}

////跳跃气缸
//int four_cylinder(uint8_t cylinder_state)
//{
//    if(cylinder_state == 0)
//    {
//    RCS_GPIO_Set(GPIOE,GPIO_Pin_6);
//		RCS_GPIO_Set(GPIOC,GPIO_Pin_13);
//		RCS_GPIO_Set(GPIOC,GPIO_Pin_14);
//		RCS_GPIO_Set(GPIOC,GPIO_Pin_15);
//			
//		return 1;

//    }
//    if(cylinder_state == 1)
//    {
//    RCS_GPIO_Reset(GPIOE,GPIO_Pin_6);
//		RCS_GPIO_Reset(GPIOC,GPIO_Pin_13);
//		RCS_GPIO_Reset(GPIOC,GPIO_Pin_14);
//		RCS_GPIO_Reset(GPIOC,GPIO_Pin_15);
//    }
//		return 0;
//	
//}




////遥控扳机控制运球
//void key_dribble(uint8_t *dribbling_active)
//{
//	if (*dribbling_active)
//    {
//        // 调用非阻塞运球函数
//        int result = RCS_Dirbble();
//        if (result == 1)  // 返回1表示运球完成
//        {
//            *dribbling_active = 0;  // 关闭运球状态
//        }
//    }
//		
//    else
//    {
//        // 检查是否需要启动运球
//        if (key_flag[1] == 1)
//        {
//            key_flag[1] = 0;
////            cstate = 0;             // 重置运球状态机
//            *dribbling_active = 1;   // 激活运球
//        }
//    }
//}

//void key_shoot(uint8_t *shooting_active)
//{
//    int speed = 0;
//    uint8_t flag1 = RCS_GPIO_Read(GPIOD, GPIO_Pin_14);
//    
//    // 检查扳机是否被按下
//    if (key_flag[1] == 1) 
//			{
//        // 扳机按下，检查光电门状态
//        if (flag1 == 0) 
//					{
//            // 光电门未触发，根据旋钮值计算速度
//            if (knob_l > 192) 
//						{
//                speed = (int)(-12000 - (knob_l - 192) * 68000 / 1600);
//            } 
//						
//						else if (knob_r > 192) 
//						{
//                speed = (int)((knob_r - 192) * 12000 / 1600);
//            } 
//						else 
//						{
//                speed = 0;
//            }
//          } 
//					else 
//				{
//            // 光电门触发，速度置0
//            speed = 0;
//        }
//        
//        // 更新电机速度
//        MotorNode_Update_Spd(speed, &shoot[0]);
//        MotorNode_Update_Spd(speed, &shoot[1]);
//        *shooting_active = (speed != 0);
//    } 
//			else 
//		{
//        // 扳机未按下，速度置0
//        MotorNode_Update_Spd(0, &shoot[0]);
//        MotorNode_Update_Spd(0, &shoot[1]);
//        *shooting_active = 0;
//    }
//}

//void jump(uint8_t *jump_active)
//{
//		if (*jump_active)
//    {
//        // 调用非阻塞跳跃函数
//        int result1 = four_cylinder(0);
//			  
//        if (result1 == 1)  // 返回1表示跳跃完成
//        {
//            *jump_active = 0;  // 关闭跳跃状态
//        }
//    }
//		
//    else
//    {
//        // 检查是否需要启动跳跃
//        if (key_flag[0] == 1)
//        {
//            key_flag[0] = 0;
//            *jump_active = 1;   // 激活跳跃
//        }
//    }
//	
//}


//void key_cylinder()
//{
//	switch(key_cylinder_state)
//	{
//		case 0:
//			if(key_flag[1] == 1)//进入运球状态
//			{
//				key_flag[1] = 0;
//				key_cylinder_state = 1;
//			}
//			else if(key_flag[0] == 1)//进入跳跃状态
//			{
//				key_flag[0] = 0;
//				key_cylinder_state = 2;
//			}
//			break;
//		case 1:
//			  // 调用非阻塞运球函数
//        
//        if (RCS_Dirbble() == 1)  // 返回1表示运球完成
//				{
//					key_cylinder_state = 0;
//				}

//				break;
//				
//		case 2:
//			if(four_cylinder(0) == 1)//跳跃
//			{
//				key_cylinder_state = 0;
//			}
//			break;
//	}
//}