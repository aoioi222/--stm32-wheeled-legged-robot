#ifndef RCS_MOTOR_REPOSISTION_H_
#define RCS_MOTOR_REPOSISTION_H_

#include "rcs.h"


#define Reset_Speed 8000.f
//typedef enum rcs_reset_err
//{
//	RCS_Reset_OK=0,
//	RCS_Reset_Continue=-1,
//	RCS_Reset_EXTI_Init_Fail=-2,//EXTI
//}RCS_Reset_Err_t;

typedef RCS_Reset_Err_t (*rcsMotor_ctrlSpd_fptr) (float);
typedef float (*rcsEncoder_getPos_fptr)(void);
int RCS_Kek_Rep_Restart(uint16_t	GPIO_Pin_x);

RCS_Reset_Err_t motor_reset_init(GPIO_TypeDef*	GPIOx,uint16_t	GPIO_Pin_x,uint8_t _priority);
RCS_Reset_Err_t motor_reset_ir(rcsMotor_ctrlSpd_fptr	ctrl_f,rcsEncoder_getPos_fptr	sensor_f,
																			GPIO_TypeDef*	GPIOx,uint16_t	GPIO_Pin_x,
																			float*	center_pos,uint8_t CAN_ID,float	LG_Angle_Distance);
RCS_Reset_Err_t motor_reset_angle_update(uint16_t	GPIO_Pin_x,uint8_t CAN_ID,float	LG_Angle_Distance,rcsEncoder_getPos_fptr	Rep_sensor_f);

#endif
