//#include "RCS_DRIBBLE.h"
//#include "UP_CTRL.h"
////运球
//uint8_t cstate;
//int RCS_Dirbble()
//{
//	
//	 switch(cstate)
//        {
//            case 0 :
//                cylinder_ctrl(0);
//						    push_ctrl(1);

//              if(Count_Delay(50,1))
//							  {
//                    cstate = 1;
//									return 0;
//							  }
//            break;
//							
//            case 1:
//                 cylinder_ctrl(1);   //爪子张开
//					       push_ctrl(1);
//            
//              if(Count_Delay(0,1))    //参数要调节  5
//                {
//                    cstate = 2;
//									return 0;
//                }
//            break;
//								
//            case 2:
//                 cylinder_ctrl(1);
//						     push_ctrl(1);       //气缸推出
//               
//              if(Count_Delay(40,1))   //  BEFORE 100
//                {
//                    cstate = 3;
//									return 0;
//                }
//            break;
//								
//            case 3 :
//                 cylinder_ctrl(1);
//						     push_ctrl(0);       //利用上一个Delay的时间，气缸收回
//                
//                if(RCS_GPIO_Read(GPIOD,GPIO_Pin_15)==0)
//                {
//                  cstate = 4;
//									return 0;
//                }
//            break;
//								
//						case 4:
//									cylinder_ctrl(1);    //爪子暂时还保持打开状态，等待球升到中间位置
//						      push_ctrl(0);
//						   if(Count_Delay(20,1))   //参数要调节
//                {
//                  cstate = 5;
//									return 0;
//                }
//						break;
//								
//						case 5:
//							cylinder_ctrl(0);     //爪子闭合
//							push_ctrl(0);
//							cstate = 0;
//							return 1;
//						break;
//									
//					}
//				
//					
//}
//		   