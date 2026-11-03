#ifndef R2_COMBCTRL_H_
#define R2_COMBCTRL_H_

#include "stdint.h"
#include "RCS_PIDctrl.h"
#include "Up_Ctrl.h"
#include "RCS_AGV_BaseMove.h"
#include "RCS_AcceCtrl.h"
#include "RCS_Stastic.h"
#include "RCS_LogServer.h"
#include "RCS_Scara.h"

#define AGV_CTRL_TYPE_SPD 0
#define AGV_CTRL_TYPE_POS 1
#define AGV_CTRL_TYPE_SRV_SPD 2
#define AGV_CTRL_TYPE_LKZ_SPD 3
#define AGV_CTRL_TYPE_SFT_SPD 4

#define GANTRY_CTRL_TYPE_POS 0
#define GANTRY_CTRL_TYPE_DIFF_POS 1

typedef struct descartes DESCARTES_AXIS;
typedef struct agv_status_vector AGV_Speed;

void RC24_FSM_Reset();

float HAL_Chassis_Get_X(void);
float HAL_Chassis_Get_Y(void);
float HAL_Chassis_Get_Z(void);

void HAL_Chassis_Reset(float x_last_output,float y_last_output,float z_last_output);
void CombCtrl_Param_Init(RCS_PIN_USART USARTx_MAP);
int8_t HAL_Scara_Action(float x_or_main,float y_or_end,uint8_t ctrl_type,uint8_t is_absolute);
int8_t HAL_Comm_Action(float gantry_pos,float actor_pos,uint8_t punk_status,uint8_t swiv_status,uint8_t gantry_ctrl_type);
void Limit_Param_Test(void);
void CombCtrl_JoyStick(void);
void CombCtrl_JoyStick_Z_Locked(uint8_t color);
int8_t FindBall(float agv_z_angle,float agv_run_spd,uint8_t zone_chose);

int8_t UpCh_Pick_BlockBall(uint8_t id);
int8_t Up_Pick_BlockBall_first(uint8_t id);
int8_t Up_Pick_BlockBall(uint8_t id);
float Get_Pos_Spd_Limit_P(float shock_pos,float limit_start_pos,float limit_start_spd,float current_pos,float current_spd);
float Get_Pos_Spd_Limit_N(float shock_pos,float limit_start_pos,float limit_start_spd,float current_pos,float current_spd);
int8_t UpCh_Pick_IsolateBall(uint8_t zone);
int8_t SZ_to_Block1(uint8_t zone,uint8_t ball_id);
int8_t SZ_to_Block2(uint8_t zone,uint8_t ball_id);
int8_t SZ_to_Block3(uint8_t zone,uint8_t ball_id);
int8_t SZ_to_Block4(uint8_t zone,uint8_t ball_id);

uint8_t HAL_Chassis_Ctrl(float param_x,float param_y,float param_z);

int8_t Block1_to_Choice(uint8_t zone);
int8_t Block2_to_Choice(uint8_t zone);
int8_t Block3_to_Choice(uint8_t zone);
int8_t Block4_to_Choice(uint8_t zone);
int8_t Isolate_to_Choice(uint8_t zone);


uint8_t Blind_Silo(uint8_t zone, uint8_t silo);
int8_t Silly_Pick(uint8_t zone);

void CombCtrl_Param_ReInit();

extern int8_t all_route_test_state;
extern int8_t test_all_flag;
void All_Route_Test(uint8_t zone);
void Laser_Reset_GPS_III(uint8_t zone);
uint8_t Laser_Reset_Or_Not(void);
void Block_Pick_Test(uint8_t zone);
#endif