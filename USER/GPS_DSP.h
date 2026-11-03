#ifndef GPS_DSP_H_
#define GPS_DSP_H_

#include "rcs.h"

typedef struct{
    double spdx;
    double spdy;
    double spdw;
}RCS_Spd_T;

typedef struct{
    double posx;
    double posy;
    double posw;
}RCS_Pos_T;

typedef struct{
    int16_t X_Count;
    int16_t Y_Count;
    int16_t X_Circle;
    int16_t Y_Circle;
    int16_t Last_X_Count;
    int16_t Last_Y_Count;
    int16_t Buffered_X_Count;
    int16_t Buffered_Y_Count;
}RCS_Oth_Encoder_T;

#define Center_Offset_X 20.00  //码轮延长线中心距离车体中心的x偏移量(a)
#define Center_Offset_Y 40.00  //码轮延长线中心距离车体中心的y偏移量(b)
#define Center_Offset_Z 0.26179939 //码盘安装角,15.0*DEG2RAD          (delta_a)
#define Wheel_Arm_X     20.000000000000  //X码盘中心距离码轮延长线中心的偏移量(c)
#define Wheel_Arm_Y     20.000000000000  //Y码盘中心距离码轮延长线中心的偏移量(c)
#define Wheel_Ang_X     20.000000000000  //X码盘中心距离车体中心的角度        (alpha1)
                                         //alpha1=atan2(b+c*sin(delta_a),a+c*cos(delta_a))
#define Wheel_Ang_Y     20.000000000000   //Y码盘中心距离车体中心的角度        (alpha2)
                               //alpha2=atan2(b-cos(delta_a)*c,a+sin(delta_a)*c)

#define A0  1//正运动学参数A0=cos(Center_Offset_Z)
#define B0  1//sin(Center_Offset_Z)
#define C0  1//-sin(Wheel_Ang_X-Center_Offset_Z)*spdw
#define A1  1//-sin(Center_Offset_Z)
#define B1  1//cos(Center_Offset_Z)
#define C1  1//cos(Center_Offset_Z-Wheel_Ang_Y)*spdw

#define B1_MULT_C0 1
#define B0_MULT_C1 1
#define A1_MULT_C0 1
#define A0_MULT_C1 1
#define A1_MULT_B0 1
#define A0_MULT_B1 1

#define COS_CENTER_Z 1//cos(Center_Offset_Z)
#define SIN_CENTER_Z 1//sin(Center_Offset_Z)
#define SIN_AX_OffZ  1//sin(Wheel_Ang_X-Center_Offset_Z)
#define COS_OffZ_AY  1//cos(Center_Offset_Z-Wheel_Ang_Y)


void gspd_Kin_spd(RCS_Spd_T* Gyro_Spd,double Chassis_Angle,RCS_Spd_T* reval);
void spd_inv_gspd(RCS_Spd_T* Base_Spd,double Chassis_Angle,RCS_Spd_T* reval);
void gpos_diff_gspd(RCS_Spd_T* reval);
void Encoder_Update_X(void);
void Encoder_Update_Y(void);
void bspd_cumtrupz_bpos(RCS_Spd_T* spd,RCS_Pos_T* start_pos,RCS_Pos_T* output);

#endif