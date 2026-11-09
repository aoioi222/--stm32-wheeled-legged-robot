#ifndef FOOT_STRUCTURE_H
#define FOOT_STRUCTURE_H

#include "robot_math.h"
#include "robot_config.h"

/* 坐标系           /\z向上
                   |
                   | /y向左
                   |/                      机头
                   +----->x向前
                  O(机体中心)
   左后腿(LEG_L_B)3              左前腿(LEG_L_F)1

   右后腿(LEG_R_B)2              右前腿(LEG_R_F)0

*/

// ====================== 关节运动学枚举与结构体定义 ======================

// 腿编号定义：F表示前，B表示后，R表示右，L表示左
typedef enum
{
    LEG_R_F = 0, // 右前腿
    LEG_L_F = 1, // 左前腿
    LEG_R_B = 2, // 右后腿
    LEG_L_B = 3  // 左后腿
} Leg_ID_t;
// 关节角度结构体/单位°
typedef struct
{
    float joint1_deg; // 髋关节展角（度）
    float joint2_deg; // 髋关节转角（度）
    float joint3_deg; // 膝关节转角（度）
    uint8_t valid;
} LegJointAngles_t;

// 足端位置结构体（单位：m，机体坐标系）
typedef struct
{
    float foot_pos_x_m; // 足端X坐标（m），机体坐标系，前=正，后=负
    float foot_pos_y_m; // 足端Y坐标（m），机体坐标系，左=正，右=负
    float foot_pos_z_m; // 足端Z坐标（m），机体坐标系，上=正，下=负
} FootPosition_t;

// 足端速度
typedef struct
{
    float foot_vel_x_mps; // X向速度 (m/s)
    float foot_vel_y_mps; // Y向速度 (m/s)
    float foot_vel_z_mps; // Z向速度 (m/s)
} FootVelocity_t;

// 足端加速度
typedef struct
{
    float foot_acc_x_mps2; // X向加速度 (m/s^2)
    float foot_acc_y_mps2; // Y向加速度 (m/s^2)
    float foot_acc_z_mps2; // Z向加速度 (m/s^2)
} FootAcceleration_t;

#endif //  FOOT_STRUCTURE_H
