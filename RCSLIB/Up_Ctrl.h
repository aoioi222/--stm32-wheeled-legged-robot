/*@filename:   Up_Ctrl.h
 *@author:     RCS16-RC
 *@brief:      上层控制
 *@date:       2024
*/

/*---------.h head-----------------------------------------*/
#ifndef _UP_CTRL_H_
#define _UP_CTRL_H_

/*---------包含文件-----------------------------------------*/

//tips:直接包括大头文件rcs.h会导致upctrl.c文件无法识别其他.h中的typedef结构体
#include "RCS_Stastic.h"
#include "RCS_Motor_Upctrl.h"
#include "RCS_PIDctrl.h"
#include "Laser_Ranging.h"
#include "stp23.h"
#include "Vision.h"

/*---------工况配置------------------------------------------*/


/*---------导出结构体----------------------------------------*/


/*---------导出函数----------------------------------------*/
//void Upctrl_Init(void);
//int Upctrl_ServoBasic(float angle_z,float pos_x,float pos_y,float force_claw);
//int Upctrl_VisionPick(float target_x,float target_y);
//void Eva_Test(float angle_z,float pos_x,float pos_y,float force_claw);
//void UpCtrl_Servo_Ctrl(float force_claw);
//void UpCtrl_Servo_UpCtrl(float _angleup1,float _angleup2,float _angleup3,float _angleget,int vision_test);
void BaseMove_joy(void);
uint32_t Point3Point_Dace(float target_x,float target_y,float target_z,DacePID_Struct* dace_pid_x,DacePID_Struct* dace_pid_y,DacePID_Struct* dace_pid_z);

uint32_t point3point_last(float target_x,float target_y,float target_z);

void Remote_control_car();
void self_circle();

void cylinder_init(void);
void cylinder_ctrl(uint8_t cylinder_state);
void push_ctrl(uint8_t cylinder_state);
void Dribble_key(void);
int four_cylinder(uint8_t cylinder_state);
void key_dribble(uint8_t *dribbling_active);
void key_shoot();
void jump(uint8_t *jump_active);
void key_cylinder();

void Myinterrupt();

/*---------.h head-----------------------------------------*/

#endif
