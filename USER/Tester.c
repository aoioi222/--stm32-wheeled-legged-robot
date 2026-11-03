//#include "Tester.h"
//int up_angle11=55,up_angle21=40,up_angle31=33;
//int up_angle12=55,up_angle22=36,up_angle32=38;
//int up_angle13=55,up_angle23=36,up_angle33=38;
//float x1=322.0f,y1=101.0f,z1=-90.0f,x2=110.0f,y2=110.0f,z2=-180.0f,x3=-110.0f, y3=-90.0f, z3=-270.0f;
////未使用变量 
//int move_angle=90,get_angle1=30,get_angle2=80,get_angle3=90,up_angle7=95,up_angle8=100,up_angle9=100,up_angle10=100; 
////底盘状态机 
////distance3代表y方向激光测距机构
////distance4代表x方向激光测距机构，特别的注意99及100状态中返回原状态的条件限幅要大于进入限幅
//int temp = 80;    //状态转换量
//int index=0;     //状态回溯量
//int reval_flag1=0;  //检测量
//int reval_flag2=0;
//int reval_flag3=0;
//int reval_flag4=0;
//int reval_flag5=0;
//int reval_flag6=0;
//int vision_position;
//void Motor_ctrl()
//{        
//	    switch(temp)
//		{
//			case 0:
//				index=temp;
//			
//			 if(point2point_last(0.0f, 0.0f, 42.0f)==1)//原地转弯 
//					temp = 1;
//				break;
//				
//				
//			case 1:
//				index=temp;
//			
//			 if(point2point_last(150.0f, 130.0f, 42.0f)==1)
//				 temp = 2;
//				break;
//				
//			 
//			case 2:
//			 index=temp;
//			
//			 if(point2point_last(180.0f, 170.0f, 0.0f)==1)//到达点位 
//				 temp = 3;
//		   break;
//			 
//			 
//			case 3:
//				index=temp;
//			
//			 if(point2point_last(180.0f, 170.0f, -90.0f)==1)//原地转弯 
//				 temp = 4;
//			break;
//			 
//			 
//			case 4:
//				index=temp;
//			
//			 if(point2point_last(322.0f, 170.0f, -90.0f)==1)//后退
//				 temp = 5;
//				break;
//			 
//			 
//			case 5:
//			index=temp;
//			 
//			 if(point2point_last(x1, y1, z1)==1)//到达第一个抓取位置 322.0f，101.0f ，-90.0f
//			 {
//				 reval_flag1=1;
//				if(Count_Delay(800,1)==1)
//				 	{ 	//1激光测距调整距离 
//						if(distance3 < 95)
//				   			temp=99;
//				   		else if(distance3>105)
//				 			temp=100;
//				  		else temp=80;
//				 	}
//		  	 }
//				break;
//			 
//			 
//			case 6:
//				index=temp;
//			 
//			 if(point2point_last(315.0f, 160.0f, -90.0f)==1)//往中场移动 
//				 temp = 7;
//				break;
//			 
//			 
//			case 7:
//			 index=temp;
//			
//			 if(point2point_last(180.0f, 170.0f, -90.0f)==1)//去第二个点位 
//				 temp = 8;
//				break;
//			 
//			 
//			case 8:
//				index=temp;
//			
//			 if(point2point_last(110.0f, 110.0f, -90.0f)==1)
//				 temp = 9;
//				break;
//			 
//			 
//			case 9:
//				index=temp;
//			 if(point2point_last(x2, y2, z2)==1)//到达第二个点位110.0f, 110.0f, -180.0f
//			 {
//				  reval_flag2=1;
//				 if(Count_Delay(800,1)==1)
//			   {
//				   temp = 505;
//				 }
//			 }
//				break;
//			 
//			 
//			case 10:
//				index=temp;
//			
//			 if(point2point_last(20.0f, 40.0f, -180.0f)==1)//??2???
//				 temp = 11;
//				break;
//			 
//			 
//		  case 11:
//				index=temp;
//			
//			 if(point2point_last(20.0f, -60.0f, -180.0f)==1)
//				 temp = 12;
//				break;
//			 
//			 
//			case 12:
//				index=temp;
//			
//			 if(point2point_last(20.0f, -60.0f, -180.0f)==1)//??6???
//				 temp = 13;
//				break;
//			 
//			 
//			case 13:
//				index=temp;
//			
//			 if(point2point_last(-110.0f, -90.0f, -180.0f)==1)
//				 temp = 14;
//				break;
//			 
//			 
//			case 14:
//			  index=temp;
//			
//			 if(point2point_last(x3, y3, z3)==1)//到达第三个点位	-110.0f, -90.0f, -270.0f
//			 {
//				  reval_flag3=1;
//			 	if(Count_Delay(800,1)==1)
//			   {
//					 
//				   temp = 506;
//				 }
//			}
//				break;
//			 
//			 
//			case 15:
//			  index=temp;
//			
//			 if(point2point_last(-270.0f, -125.0f, -270.0f)==1)//??7???
//				 temp = 16;
//				break;
//			 
//			 
//			 	case 16:
//			  index=temp;
//			
//			  if(point2point_last(-270.0f, -80.0f, -270.0f)==1)//??8???
//				 temp = 507;//激光测距调整距离 并开始放置物块 
//				break;
//			 
//			 /////////////////////////////////////回到启动区 
//			 case 17:
//			  index=temp;
//			
//			 if(point2point_last(-350.0f, -90.0f, -270.0f)==1)//?????
//				 temp = 18;
//				break;
//			 
//			 
//			 case 18:
//			  index=temp;
//			
//			 if(point2point_last(-350.0f, -90.0f, -360.0f)==1)//?????
//				 temp = 19;
//				break;
//			 
//			 
//			 case 19:
//			  index=temp;
//			
//			 if(point2point_last(-350.0f, -125.0f, -360.0f)==1)//?????
//				 temp = 20;
//				break;
//			 
//			 
//			case 20:
//				BaseMove_P2P( 0,0,0);
//        break;
//			
//		//50songkai 90zhuaqu 
//			case 64:
//				//转到收纳区上方，放置中间物块 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(15,106,80,90);
//				if(Count_Delay(500,1)==1)
//        {
//				 
//          //temp=65;	
//				}
//			break;
//			case 65:
//				//放下物块 

//				BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(15,100,80,50);
//				if(Count_Delay(500,1) == 1)
//        {
//				 
//          temp=66;	
//				}
//			break;
//			break;
//				
//			///////////////////////////////////////////放置左边物块	
//				case 66:
//				//
//			  BaseMove_P2P( 0,0,0);
//		Upctrl_ServoBasic(123,107,60,50);
//				if(Count_Delay(800,1)==1)
//        {
//				  
//          temp=67;	
//				}	
//			break;
//				
//			case 67:
//				//	下爪 抓取 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(123,107,53,90);
//				if(Count_Delay(800,1) == 1)
//        {
//				 
//          temp=167;	
//				}
//			break;
//				case 167:
//				//	???????:?????? 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(123,110,80,90);
//				if(Count_Delay(800,1) == 1)
//        {
//				 
//          temp=68;	
//				}
//			break;
//			case 68:
//				//移动到收纳区上方 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(17,100,80,90);
//				if(Count_Delay(500,1)==1)
//        {
//				 
//          temp=69;	
//				}
//			break;
//			case 69:
//				//松爪 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(15,100,80,50);
//				if(Count_Delay(500,1) == 1)
//        {
//				 
//          temp=70;	
//				}
//			break;
//				
//				///////////////////////////////////////////////////放置右边物块 
//				
//				case 70:
//				//???????:?????
//			  BaseMove_P2P( 0,0,0);
//		Upctrl_ServoBasic(63,103,52,90);
//				if(Count_Delay(800,1)==1)
//        {
//          temp=71;	
//				}	
//			break;
//				
//			case 71:
//				//	???????:?????? 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(63,103,52,90);
//				if(Count_Delay(800,1) == 1)
//        {
//				 
//          temp=172;	
//				}
//			break;
//				case 172:
//				//	???????:?????? 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(63,105,80,90);
//				if(Count_Delay(800,1) == 1)
//        {
//				 
//          temp=72;	
//				}
//			break;
//			case 72:
//				//??????3:?????????????
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(15,100,80,90);
//				if(Count_Delay(500,1)==1)
//        {
//				 
//          temp=73;	
//				}
//			break;
//			case 73:
//				//??????4:????
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(15,100,80,50);
//				if(Count_Delay(500,1) == 1)
//        {
//				 
//          temp=74;	
//				}
//			break;
//				
//			case 74:
//				//回到原位 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(90,90,90,50);
//				if(Count_Delay(500,1) == 1)
//        {
//				 
//          //temp=index+1;	
//				}
//			break;
//				
//			
//				
//				
//				
//				
//					//50songkai 90zhuaqu 
//				//抓取物块1
//				//////////////////////////////////////////////////////////
//			case 80:
//				//抬起机械臂
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(15,140,110,get_angle1);
//				vision_position=Vision_identify();
//				if(Count_Delay(800,1) == 1)
//				{   //判断物块位置，抓取不用位置的物块
//					  if(vision_position==1)
//               temp=81;	
//						if(vision_position==2)
//							temp=181;
//						if(vision_position==3)
//							temp=281;		
//				}
//			break;
//		//抓取position1
//			case 81:
//			//视觉识别
//			  BaseMove_P2P( 0,0,0);
//			Upctrl_VisionPick(230.f,110.f);
//				if(Count_Delay(1500,1)== 1)
//        {
//          temp=82;	
//					
//				}
//			break;
//				
//			case 82:
//			//下爪抓取
//			  BaseMove_P2P( 0,0,0);
//			UpCtrl_Servo_UpCtrl(up_angle11,up_angle21,up_angle31,get_angle2,vision_position);
//				if(Count_Delay(800,1)==1)
//        {
//				  
//          temp=83;	
//				}	
//			break;
//				
//			case 83:
//			//	抬起
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(15,100,get_angle2,90);
//				if(Count_Delay(500,1) == 1)
//        {
//					
//				 if(reval_flag1==1)
//				 {
//         			 temp=84;	//放置左侧
//					reval_flag1=0;
//				 }
//				 
//				 else if(reval_flag2==1)
//				 {
//				 temp=104;	//放置右侧
//				reval_flag2=0;
//				 }
//				 
//				  else if(reval_flag3==1)
//				 {
//				 temp=94;	//抓取第三块物块
//				reval_flag3=0;
//				 }
//				}
//			break;
//				////////////////////////
//				////抓取position2
//				case 181:
//			//识别
//			  BaseMove_P2P( 0,0,0);
//			 Upctrl_VisionPick(230.f,110.f);
//				if(Count_Delay(1500,1)== 1)
//        {
//          temp=182;	
//					
//				}
//			break;
//			case 182:
//			//下爪
//			  BaseMove_P2P( 0,0,0);
//		  	UpCtrl_Servo_UpCtrl(up_angle12,up_angle22,up_angle32,90,vision_position);
//				if(Count_Delay(800,1)==1)
//        {
//				  
//          temp=183;	
//				}	
//			break;
//				
//			case 183:
//			//	识别
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(15,100,80,90);
//				if(Count_Delay(500,1) == 1)
//       				 {
//					
//						 if(reval_flag1==1)
//							 {
//          						temp=84;	
//								reval_flag1=0;
//							 }
//				 
//						 else if(reval_flag2==1)
//							 {
//				 				temp=104;	
//								reval_flag2=0;
//				 			}
//				 
//				  		else if(reval_flag3==1)
//							 {
//								 temp=94;	
//								reval_flag3=0;
//							 }
//					}
//			break;
//			//////////////////////////////	
//				//抓取position3
//			case 281:
//			//???????:????????? 
//			  BaseMove_P2P( 0,0,0);
//			Upctrl_VisionPick(230.f,110.f);
//				if(Count_Delay(1500,1)== 1)
//        {
//          temp=282;	
//					
//				}
//			break;
//			case 282:
//			//???????:????
//			  BaseMove_P2P( 0,0,0);
//		  	UpCtrl_Servo_UpCtrl(up_angle13,up_angle23,up_angle33,90,vision_position);
//				if(Count_Delay(800,1)==1)
//        {
//				  
//          temp=283;	
//				}	
//			break;
//				
//			case 283:
//			//	????? 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(15,100,80,90);
//				if(Count_Delay(500,1) == 1)
//        {
//					
//				 if(reval_flag1==1)
//				 {
//          temp=84;	
//					reval_flag1=0;
//				 }
//				 
//				 else if(reval_flag2==1)
//				 {
//				 temp=104;	
//				reval_flag2=0;
//				 }
//				 
//				  else if(reval_flag3==1)
//				 {
//				 temp=94;	
//				reval_flag3=0;
//				 }
//				}
//			break;
//				//////////////////////////////////////????
//			case 84:
//				//??????3:?????? 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(125,100,80,90);
//				if(Count_Delay(800,1)==1)
//        {
//				 
//          temp=85;	
//				}
//			break;
//			case 85:
//				//??????4:??????
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(125,110,55,50);
//				if(Count_Delay(400,1) == 1)
//        {
//				 
//          temp=86;	
//				}
//			break;
//			case 86:
//				//??????5:??????? 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(125,100,80,50);
//				if(Count_Delay(500,1) == 1)
//        {
//				 
//          temp=87;	
//				}
//			break;
//				case 87:
//				//??
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(90,90,90,50);
//				if(Count_Delay(500,1) == 1)
//        {
//				 
//          //temp=index+1;	
//				}
//			break;
//				
//				
//				///////////////////////////////////////////////????
//						case 94:
//				//??????3:??????
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(90,100,100,90);
//				if(Count_Delay(800,1)==1)
//        {
//				 
//          //temp=temp=index+1;	
//				}
//			break;

//					case 104:
//				//??????3:??????
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(66,100,80,90);
//				if(Count_Delay(800,1)==1)
//        {
//				 
//          temp=105;	
//				}
//			break;
//			case 105:
//				//??????4:??????
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(66,110,55,50);
//				if(Count_Delay(400,1) == 1)
//        {
//				 
//          temp=106;	
//				}
//			break;
//			case 106:
//				//??????5:??????? 
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(66,100,80,50);
//				if(Count_Delay(500,1) == 1)
//        {
//				 
//          temp=107;	
//				}
//			break;
//				
//				
//				case 107:
//				//??
//			  BaseMove_P2P( 0,0,0);
//				Upctrl_ServoBasic(90,90,90,50);
//				if(Count_Delay(500,1) == 1)
//        {
//				 
//          //temp=index+1;	
//				}
//			break;
//				
//				
//			//////////////////////////////////////////////////////激光测距调整距离
//				
//			case 99:
//				 BaseMove_P2P( 0,-100,0);
//			  if(distance3>95&&distance3<105)
//			   temp=80;
//				
//				
//			  break;
//			
//			case 100:
//				 BaseMove_P2P( 0,100,0);
//			if(distance3>95&&distance3<105)
//			   temp=80;
//			  break;
//			
//			 case 501:
//				BaseMove_P2P(-100,0,0);
//			  if(distance4<805&&distance4>795)
//					temp = 505;
//				break;
//			case 502:
//				BaseMove_P2P(100,0,0);
//			  if(distance4<805&&distance4>795)
//					temp = 505;
//				break;
//      case 503:
//        BaseMove_P2P(0,-100,0);
//        if(distance3>95&&distance3<105)
//					temp = 505;
//				break;
//      case 504:
//        BaseMove_P2P(0,100,0);
//			  if(distance3>95&&distance3<105)
//					temp = 505;
//				break;			
//			case 505:
//				 if(distance4<795)
//				    temp=501;
//			    else if(distance4>805)
//   		      temp=502;
//				  else if(distance3<95)
//					 temp = 503;
//				  else if(distance3>105)
//					 temp = 504;
//					else
//						temp=80;
//				break;
//			case 506:
//				  if(distance4<650)
//				    temp=501;
//			    else if(distance4>750)
//   		      temp=502;
//				  else if(distance3<95)
//					 temp = 503;
//				  else if(distance3>105)
//					 temp = 504;
//					else
//						temp=80;
//				break;
//			case 507:
//				  if(distance4<650)
//				    temp=501;
//			    else if(distance4>750)
//   		      temp=502;
//				  else if(distance3<95)
//					 temp = 503;
//				  else if(distance3>105)
//					 temp = 504;
//					else
//						temp=64;
//				break;
//		}
//}
