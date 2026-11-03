#ifndef FOOT_MODE_CONTRL
#define FOOT_MODE_CONTRL

#include "stm32f4xx.h"
#include "rcs.h"

/* 舵机数量: 4腿 × 3关节 = 12 */
#define SERVO_COUNT         12

/* HTS-35H 串行总线舵机参数 */
#define SERVO_ANGLE_RANGE   240.0f
#define SERVO_POS_RANGE     1000.0f
#define SERVO_POS_MID       500.0f
#define SERVO_MOVE_TIME     20//若实测舵机响应不够，把 SERVO_BAUD 提到 1Mbps 并同步改舵机端波特率

/* 舵机总线: USART3 半双工, 115200bps */
#define SERVO_USART         USART3
#define SERVO_GPIO          GPIOD
#define SERVO_PIN_TX        GPIO_Pin_8
#define SERVO_PIN_RX        GPIO_Pin_9
#define SERVO_BAUD          1000000

/* 舵机ID映射: 腿顺序 RF(右前), LF(左前), RB(右后), LB(左后) */
/* 每腿关节顺序: 髋外展, 髋旋转, 膝 */
#define SERVO_ID_RF_HIP_ABD   1
#define SERVO_ID_RF_HIP_ROT   2
#define SERVO_ID_RF_KNEE      3
#define SERVO_ID_LF_HIP_ABD   4
#define SERVO_ID_LF_HIP_ROT   5
#define SERVO_ID_LF_KNEE      6
#define SERVO_ID_RB_HIP_ABD   7
#define SERVO_ID_RB_HIP_ROT   8
#define SERVO_ID_RB_KNEE      9
#define SERVO_ID_LB_HIP_ABD   10
#define SERVO_ID_LB_HIP_ROT   11
#define SERVO_ID_LB_KNEE      12

/* 关节限位 (°) */
#define HIP_ABD_MIN    -120.0f
#define HIP_ABD_MAX     120.0f
#define HIP_ROT_MIN     -90.0f
#define HIP_ROT_MAX      90.0f
#define KNEE_MIN       -150.0f
#define KNEE_MAX         0.0f

/* 关节类型枚举 */
typedef enum {
    JOINT_HIP_ABD,
    JOINT_HIP_ROT,
    JOINT_KNEE
} JointType_t;

void Foot_Mode_Init(void);
void Foot_Mode_Control(const float joint_angle[SERVO_COUNT]);
void Foot_Mode_Stop(void);
float clamp_joint_angle(JointType_t joint_type, float input_angle);

#endif
