#include "R2_Upctrl.h"
#define angle_open -3390//2006旋转角度-开阀
#define angle_close 0//2006旋转角度-闭阀

//typedef struct agv_multi_coord_param{
//	float      Agv_Bridge_Drv_Spd;
//	float      Agv_Bridge_Srv_Spd;
//	float      Agv_Bridge_StateOut_Pos;

//	AGV_Speed  Agv_Output;
//	DacePID_Struct Agv_Pos_Pid_X;
//	DacePID_Struct Agv_Pos_Pid_Y;
//	DacePID_Struct Agv_Pos_Pid_Z;
//	uint8_t    Agv_Multi_Fsm;

//	float      tmp_x_out;
//	float      tmp_y_out;
//	float      tmp_vec_out;
//	float      tmp_z_out;
//	AGV_Speed  tmp_agv_output;
//	float      tmp_agv_srv_err;
//}AGV_MultiCoord_Param;

//typedef enum agv_mcl_enum{
//	Mode_Drver =0,
//	Mode_Bridge=1,
//}AGV_MultiCoord_Enum;

//int8_t AGV_Multi_CoordLine_Ctrl(float target_x,float target_y,float fixed_z,AGV_MultiCoord_Param* hdl)
//{
//	//??????
//	switch(hdl->Agv_Multi_Fsm)
//	{
//		//??pid?????????????,???????
//		//??:????????????PID???
//		case Mode_Drver:
//			hdl->tmp_x_out=PID_Normal_Ctrl(target_x,HAL_Chassis_Get_X(),&(hdl->Agv_Pos_Pid_X.pid));
//			hdl->tmp_y_out=PID_Normal_Ctrl(target_y,HAL_Chassis_Get_Y(),&(hdl->Agv_Pos_Pid_Y.pid));
//			hdl->tmp_vec_out=sqrtf(hdl->tmp_x_out*hdl->tmp_x_out+hdl->tmp_y_out*hdl->tmp_y_out);
//			if (hdl->tmp_vec_out <= hdl->Agv_Bridge_Drv_Spd) 
//			{
//				hdl->Agv_Multi_Fsm=Mode_Bridge;
//			}
//		break;

//		//??????????????????????,?????????
//		case Mode_Bridge:
//			hdl->tmp_agv_output=AGV_Re_Center_Move_v33_Cac(target_x,target_y,0);
//			hdl->tmp_agv_srv_err=fabsf(hdl->tmp_agv_output.angle[0]-hdl->Agv_Output.angle[0]);
//			if (hdl->tmp_agv_srv_err <= hdl->Agv_Bridge_StateOut_Pos )
//			{
//				hdl->Agv_Multi_Fsm=Mode_Drver;
//			}
//		break; 
//	}

//	//??????
//	switch(hdl->Agv_Multi_Fsm)
//	{
//		case Mode_Drver:
//			HAL_Chassis_Ctrl(hdl->tmp_x_out,hdl->tmp_y_out,hdl->tmp_z_out,AGV_CTRL_TYPE_POS);
//			hdl->Agv_Output=AGV_Get_Last_Spd();
//			return Mode_Drver;
//		break;

//		case Mode_Bridge:
//			HAL_Chassis_Ctrl(target_x,target_y,hdl->Agv_Bridge_Srv_Spd,AGV_CTRL_TYPE_SFT_SPD);
//			hdl->Agv_Output=AGV_Get_Last_Spd();
//			return Mode_Bridge;
//		break;
//	}
//}
////????????,???????????
//float Vision_Pick_ValidCheck_Main(uint8_t window_size)
//{
//	DESCARTES_AXIS current_ball_pos;
//	float temp_input_y;
//	float temp_input_x;
//	
//	//????
//	current_ball_pos=Vision_Get_Ball_Pos();
//	WindowFloat_Update_Size(&Vision_Pick_Window_X,window_size);
//	WindowFloat_Update_Size(&Vision_Pick_Window_Y,window_size);
//	
//	//?????????
//	temp_input_x=(float)(current_ball_pos.x);
//	temp_input_y=(float)(current_ball_pos.y);
//	WindowFloat_Update_Member(&Vision_Pick_Window_X,temp_input_x);
//	WindowFloat_Update_Member(&Vision_Pick_Window_Y,temp_input_y);
//}

////????????????
//float valid_vision_pick_xy[3][30];
//float valid_vision_pick_x_mean;
//float valid_vision_pick_y_mean;
//void Vision_Pick_Valid_Get_Performance(float* valid_percent,float* var_xy)
//{
//	static uint8_t valid_vision_count;
//	float temp_float_x;
//	float temp_float_y;
//	
//	int i;
//	float valid_variance;
//	float valid_mean_x;
//	float valid_mean_y;
//	float valid_percent_count;
//	
//	if (WindowFloat_Get_Redy_Flag(&Vision_Pick_Window_X))
//	{
//		valid_vision_count=0;
//		//??????,??????
//		for(i=0;i<Vision_Pick_Window_X.window_size;i++)
//		{
//			//?????
//			memcpy(&temp_float_x,WindowFloat_Get_Member_Ptr(&Vision_Pick_Window_X,i),sizeof(float));
//			memcpy(&temp_float_y,WindowFloat_Get_Member_Ptr(&Vision_Pick_Window_Y,i),sizeof(float));
//			//??????	
//			if ((temp_float_x==0.0f)&&(temp_float_y==0.0f))
//			{
//				//????????
//				__NOP();
//			}
//			else
//			{
//				//?????????
//				valid_vision_pick_xy[0][valid_vision_count]=temp_float_x;
//				valid_vision_pick_xy[1][valid_vision_count]=temp_float_y;
//				valid_vision_pick_xy[2][valid_vision_count]=temp_float_x+temp_float_y;
//				valid_vision_count++;
//			}		
//		}
//		
//		//???????????
//		valid_variance=ArrayFloat_Get_Var(valid_vision_pick_xy[2],valid_vision_count);
//		valid_mean_x=ArrayFloat_Get_Mean(valid_vision_pick_xy[0],valid_vision_count);
//		valid_mean_y=ArrayFloat_Get_Mean(valid_vision_pick_xy[1],valid_vision_count);
//		
//		//????????????
//		if (Vision_Pick_Window_X.window_size!=0) valid_percent_count=(1.0f*valid_vision_count)/(1.0f*Vision_Pick_Window_X.window_size);else valid_percent_count=1.0f;
//		*valid_percent=valid_percent_count;
//		*var_xy=valid_variance;
//		valid_vision_pick_x_mean=valid_mean_x;
//		valid_vision_pick_y_mean=valid_mean_y;
//	}
//}

////???????????????
//void Vision_Pick_Valid_Get_Data(float* mean_x,float* mean_y)
//{
//	*mean_x=valid_vision_pick_x_mean;
//	*mean_y=valid_vision_pick_y_mean;
//}

///**
// * @name: FindBall
// * @brief:?????????????
// **/
//int8_t         state_findball;   //?????
//float          target_main;      //??????
//SCARA_AXIS     scara_motor_pos;  //?????????scara??
//DESCARTES_AXIS scara_desc_pos;   //??????????????
//SCARA_AXIS     scara_seeked_pos; //???????scara??
//volatile float window_valid_percent;//???????????
//volatile float window_variance_x;   //x??
//volatile float window_variance_d;   //d??xxxxx
//volatile float window_valid_x;      //x??
//volatile float window_valid_d;      //d??xxxxxx
//volatile float window_valid_y;      //y??vvvvvv
//int8_t ComboCtrl_FindBall(float agv_z_angle,float agv_run_spd,uint8_t zone_chose)
//{
//	//????
//	int8_t     finish_flag[4];
//	char log_addl[60];
//	
//	switch(state_findball)
//	{
//		//-----------------????????????(???????,????)------------------------------
//		case 0:
//			finish_flag[0]=HAL_Comm_Action(GANTRY_FINDBALL_POS,ACTOR_UP_POS,0,0);
//			finish_flag[1]=HAL_Scara_Action(150.0f,SCARA_END_FOLD_POS_DEG,2,RELATIVE_SCARA);//???2???:????????
//			finish_flag[2]=HAL_Chassis_Ctrl(0.0f,0.0f,agv_z_angle,3);                   //??3???:xy????,z????

//			if (finish_flag[0]==1 && finish_flag[1]==1 && finish_flag[2]==1) 
//			{
//				LogServer_Collect(&Combo_Log,"E","FIND_BALL:prepared finish!","CASE 0 -> CASE 1");
//				state_findball=1;
//			}
//		break;

//		//-----------------------??????,??????????----------------------------------------
//		case 1:
//			finish_flag[0]=HAL_Comm_Action(GANTRY_FINDBALL_POS,ACTOR_UP_POS,0,0);
//			finish_flag[1]=HAL_Scara_Action(200.0f,0.0f,3,RELATIVE_SCARA);//???3???:????????
//			finish_flag[2]=HAL_Chassis_Ctrl(0.0f,0.0f,agv_z_angle,3);     //??3???:xy????,z????

//			//?????,????
//			if (Vision_Cup_Get_Ball_Pos_X()!=0 && Vision_Cup_Get_Ball_Pos_Y()!=0) 
//			{
//				LogServer_Collect(&Combo_Log,"E","FIND_BALL:ball decteced!","CASE 1 -> CASE 3");
//				state_findball=3;
//			}
//			//??????,????
//			scara_motor_pos=R2_Scara_Get_SPos();
//			if (scara_motor_pos.joint_angle_main>= (135.0f*DEG2RAD)) state_findball=2;
//		break;

//		case 2:
//			HAL_Comm_Action(GANTRY_FINDBALL_POS,2000.0f,0,0);
//			HAL_Scara_Action(-200.0f,0.0f,3,RELATIVE_SCARA);
//			HAL_Chassis_Ctrl(0.0f,0.0f,agv_z_angle,3);

//			//?????,????
//			if (Vision_Cup_Get_Ball_Pos_X()!=0 && Vision_Cup_Get_Ball_Pos_Y()!=0) 
//			{
//				LogServer_Collect(&Combo_Log,"E","FIND_BALL:ball decteced!","CASE 2 -> CASE 3");
//				state_findball=3;
//			}
//			//??????,????
//			
//			scara_motor_pos=R2_Scara_Get_SPos();
//			if (scara_motor_pos.joint_angle_main<= (45.0f*DEG2RAD)) state_findball=1;
//		break;
//		
//		//---------------------------??PID??,??????----------------------------------------------
//		case 3:
//			//??????????
//			Vision_Cup_ValidCheck_Main(Cup_Window_Len);
//			Vision_Cup_Valid_Get_Performance(&window_valid_percent,&window_variance_x,&window_variance_d);
//			Vision_Cup_Valid_Get_Data(&window_valid_x,&window_valid_y,&window_valid_d);

//			//??????????,????????,???????
//			if (window_valid_percent >= Cup_Tor_Valid_Percent &&  //10????5??????? 
//				  window_variance_x    <= Cup_Tor_Valid_Var_X)      //???????????15
//			    
//			{
//				target_main=PID_Normal_Ctrl(Cup_Center_Pixcel,
//				                            window_valid_x,
//				                            &FindBall_Main_Pid);
//			}
//			else
//			{
//				target_main=0;
//				sprintf(log_addl,"Percent=%d,Var_X=%d,Var_D=%d",10.0f*window_valid_percent,window_variance_x,window_variance_d);
//				LogServer_Collect(&Combo_Log,"ES","FIND_BALL:Invalid Vision_Cup Data!",log_addl);
//			}
//			
//			//???????
//			HAL_Comm_Action(GANTRY_FINDBALL_POS,2000.0f,0,0);
//			HAL_Scara_Action(target_main,0.0f,3,RELATIVE_SCARA);
//			HAL_Chassis_Ctrl(0.0f,0.0f,agv_z_angle,3);

//			//?????????
//			if (abs(Cup_Center_Pixcel-Vision_Cup_Get_Ball_Pos_X())<=5) 
//			{
//				state_findball++;
//				LogServer_Collect(&Combo_Log,"E","FIND_BALL:Camera Aimed Finished!","CASE 3 -> CASE 4");
//			}
//		break;

//		//-----------------????,???????,??????,????????------------------------------
//		case 4:
//			//??????????
//			Vision_Cup_ValidCheck_Main(10);
//			Vision_Cup_Valid_Get_Performance(&window_valid_percent,&window_variance_x,&window_variance_d);
//			Vision_Cup_Valid_Get_Data(&window_valid_x,&window_valid_y,&window_valid_d);

//			//??????????,????????,???????
//			if (window_valid_percent >= Cup_Tor_Valid_Percent && 
//				  window_variance_x    <= Cup_Tor_Valid_Var_X)
//			{
//				target_main=PID_Normal_Ctrl(Cup_Center_Pixcel,window_valid_x,&FindBall_Main_Pid);
//				scara_motor_pos=R2_Scara_Get_SPos();

//				R2_Comm_Action(GANTRY_FINDBALL_POS,2000.0f,0,0);
//				HAL_Scara_Action(target_main,0.0f,3,RELATIVE_SCARA);
//				HAL_Chassis_Ctrl(agv_run_spd*cosf(scara_motor_pos.joint_angle_main-60.0f*DEG2RAD),
//					             agv_run_spd*sinf(scara_motor_pos.joint_angle_main-60.0f*DEG2RAD),
//					             agv_z_angle,
//					             3);
//			}
//			else
//			{
//				R2_Comm_Action(GANTRY_FINDBALL_POS,2000.0f,0,0);
//				HAL_Scara_Action(0.0f,0.0f,3,RELATIVE_SCARA);
//				HAL_Chassis_Ctrl(0.0f,0.0f,agv_z_angle,3);

//				sprintf(log_addl,"Percent=%d,Var_X=%d,Var_D=%d",10.0f*window_valid_percent,window_variance_x,window_variance_d);
//				LogServer_Collect(&Combo_Log,"ES","FIND_BALL:Invalid Vision_Cup Data!",log_addl);
//			}		
//			

//			//?????????30,????USB?????
//			if ((window_valid_d <= 30.0f) && (window_valid_d != 0.0f)) //mod
//			{
//				state_findball=6;
//				scara_desc_pos=R2_Scara_Get_Pos();//??????????,?????????
//				//scara_seak_main=scara_desc_pos.x*SCARA_MAX_SIZE;
//				//scara_seak_end=scara_desc_pos.y*SCARA_MAX_SIZE;
//				LogServer_Collect(&Combo_Log,"ES","FIND_BALL:Cup Camera Finish its Task!","CASE 4 -> CASE 6");
//			}

//			//???,????5??

//		break;
//			
//		case 5:
//			
//		break;

//		//--------------------------????,???????USB???????-------------------------------------
//		case 6:
//			//err!
//			//Vision_Pick_ValidCheck_Main(15);
//			//Vision_Cup_Valid_Get_Performance(&window_valid_percent,&window_variance_x,RELATIVE_SCARA);

//			HAL_Comm_Action(GANTRY_FINDBALL_POS,200.0f,0,0);
//			HAL_Scara_Action(0,
//			                 0,                //reach to max
//			                 1,
//			                 RELATIVE_SCARA);               //???0???:????????
//			HAL_Chassis_Ctrl(0.0f,0.0f,agv_z_angle,3);     //??3???:xy????,z????

//			//????????,????,????7????
////			if (window_valid_percent <= 0.3f)
////			{
////				//state_findball=7;
////				//LogServer_Collect(&Combo_Log,"ES","FIND_BALL:Pick Camera Could Not Veriy Target!",NULL);
////			}
//		break;

//		case 7:
//			
//		break;
//	}
//}


//int8_t ComboCtrl_FindBall_V2(float agv_z_angle,float agv_run_spd,uint8_t zone_chose)
//{
//	//????
//	int8_t finish_flag[4];
//	char  log_addl[60];
//	
//	switch(state_findball)
//	{
//		//-----------------???????????????-----------------------------------------------
//		case 0:

//			finish_flag[0]=HAL_Comm_Action(GANTRY_FINDBALL_POS,ACTOR_UP_POS,0,0);
//			finish_flag[1]=HAL_Scara_Action(150.0f,SCARA_END_FOLD_POS_DEG,2,RELATIVE_SCARA);
//			
//			finish_flag[2]=HAL_Chassis_Ctrl(0.0f,0.0f,agv_z_angle,3);                  


//		break;

//		
//		
//		//-----------------????,???????,??????,????????------------------------------
//		case 3:
//			//??????????
//			Vision_Cup_ValidCheck_Main(10);
//			Vision_Cup_Valid_Get_Performance(&window_valid_percent,&window_variance_x,&window_variance_d);
//			Vision_Cup_Valid_Get_Data(&window_valid_x,&window_valid_y,&window_valid_d);

//			//??????????,????????,???????
//			if (window_valid_percent >= Cup_Tor_Valid_Percent && 
//				  window_variance_x    <= Cup_Tor_Valid_Var_X)
//			{
//				target_main=PID_Normal_Ctrl(Cup_Center_Pixcel,window_valid_x,&FindBall_Main_Pid);
//				R2_Comm_Action(GANTRY_FINDBALL_POS,ACTOR_UP_POS,0,0);
//				HAL_Scara_Action(0.0f,0.0f,3,RELATIVE_SCARA);
//				HAL_Chassis_Ctrl(0.0f,target_main,agv_z_angle,3);
//			}
//			else
//			{
//				R2_Comm_Action(GANTRY_FINDBALL_POS,ACTOR_UP_POS,0,0);
//				HAL_Scara_Action(0.0f,0.0f,3,RELATIVE_SCARA);
//				HAL_Chassis_Ctrl(0.0f,0.0f,agv_z_angle,3);

//				sprintf(log_addl,"Percent=%d,Var_X=%d,Var_D=%d",10.0f*window_valid_percent,window_variance_x,window_variance_d);
//				LogServer_Collect(&Combo_Log,"ES","FIND_BALL:Invalid Vision_Cup Data!",log_addl);
//			}		
//			

//		//--------------------------????,???????USB???????-------------------------------------
//		case 6:
//			//err!
//			//Vision_Pick_ValidCheck_Main(15);
//			//Vision_Cup_Valid_Get_Performance(&window_valid_percent,&window_variance_x,RELATIVE_SCARA);

//			HAL_Comm_Action(GANTRY_FINDBALL_POS,200.0f,0,0);
//			HAL_Scara_Action(0,
//			                 0,                //reach to max
//			                 1,
//			                 RELATIVE_SCARA);               //???0???:????????
//			HAL_Chassis_Ctrl(0.0f,0.0f,agv_z_angle,3);     //??3???:xy????,z????

//			//????????,????,????7????
////			if (window_valid_percent <= 0.3f)
////			{
////				//state_findball=7;
////				//LogServer_Collect(&Combo_Log,"ES","FIND_BALL:Pick Camera Could Not Veriy Target!",NULL);
////			}
//		break;

//		case 7:
//			
//		break;
//	}
//}
