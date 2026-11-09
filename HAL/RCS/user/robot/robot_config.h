#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "robot_math.h"



/*============================================================================
 * 机器人全局参数配置文件
 * 集中管理所有可调参数，按类别组织
 *============================================================================*/

/*============================================================================
 * 一、仿真与控制周期
 *============================================================================*/
#define TIME_STEP 5        // 仿真步长 (ms)
#define MOTOR_VELOCITY 5.0f // 电机最大速度，步态用 (rad/s)
#define INIT_WAIT_TIME 1.5f // 初始化归位等待时间 (s)

/*============================================================================
 * 二、机械结构尺寸 (单位: 米)
 *============================================================================*/

// 连杆长度
#define HIP_ABD_LENGTH 0.023f // 髋展连杆长度 (Joint1→Joint2 Y方向偏移)
#define THIGH_LENGTH 0.10f    // 大腿长度 (Joint2→Joint3)
#define CALF_LENGTH 0.10f     // 小腿长度 (Joint3→足端)
#define FULL_LEN (THIGH_LENGTH + CALF_LENGTH)

// 髋关节安装位置偏移 (机体坐标系)
#define HIP_OFFSET_X_R_F 0.17f   // 右前髋 X (前+)
#define HIP_OFFSET_Y_R_F -0.065f // 右前髋 Y (右-)
#define HIP_OFFSET_Z_R_F 0.01f   // 右前髋 Z (上+)

#define HIP_OFFSET_X_L_F 0.17f  // 左前髋 X
#define HIP_OFFSET_Y_L_F 0.065f // 左前髋 Y (左+)
#define HIP_OFFSET_Z_L_F 0.01f  // 左前髋 Z

#define HIP_OFFSET_X_R_B -0.17f  // 右后髋 X (后-)
#define HIP_OFFSET_Y_R_B -0.065f // 右后髋 Y
#define HIP_OFFSET_Z_R_B 0.01f   // 右后髋 Z

#define HIP_OFFSET_X_L_B -0.17f // 左后髋 X
#define HIP_OFFSET_Y_L_B 0.065f // 左后髋 Y
#define HIP_OFFSET_Z_L_B 0.01f  // 左后髋 Z

// 足端默认位置偏移
#define FOOT_OFFSET_X 0.17f   // 足端 X 偏移 (前后)
#define FOOT_OFFSET_Y 0.078f  // 足端 Y 偏移 (左右)0.088f,为保证内测，减小
#define FOOT_OFFSET_Z 0.1538f // 足端 Z 偏移 (站立时向下为负)

/*============================================================================
 * 重力补偿参数
 *============================================================================*/
#define HIP_ABD_MASS 0.05f    // 髋外展连杆质量 (kg)，占位值，需实测
#define THIGH_MASS   0.20f    // 大腿质量 (kg)，占位值，需实测
#define CALF_MASS    0.15f    // 小腿质量 (kg)，占位值，需实测
#define GRAV_COMP_GAIN 0.0f   // 重力补偿前馈增益 (deg/(N·m))，0=关闭，标定后填正值
/* 编译开关：预处理器表达式不支持浮点常量，所以不能用 #if GRAV_COMP_GAIN 判断，
   要开启重力补偿就把这里改成 1，同时把 GRAV_COMP_GAIN 填成标定值 */
#define GRAV_COMP_ENABLE 0

// 足端站立初始位置 (由偏移量派生)
#define LEG_R_F_START_POS_X FOOT_OFFSET_X
#define LEG_R_F_START_POS_Y (-FOOT_OFFSET_Y)
#define LEG_R_F_START_POS_Z (-FOOT_OFFSET_Z)

#define LEG_L_F_START_POS_X FOOT_OFFSET_X
#define LEG_L_F_START_POS_Y FOOT_OFFSET_Y
#define LEG_L_F_START_POS_Z (-FOOT_OFFSET_Z)

#define LEG_R_B_START_POS_X (-FOOT_OFFSET_X)
#define LEG_R_B_START_POS_Y (-FOOT_OFFSET_Y)
#define LEG_R_B_START_POS_Z (-FOOT_OFFSET_Z)

#define LEG_L_B_START_POS_X (-FOOT_OFFSET_X)
#define LEG_L_B_START_POS_Y FOOT_OFFSET_Y
#define LEG_L_B_START_POS_Z (-FOOT_OFFSET_Z)

// 关节限位 (度)
#define JOINT1_MIN -30.0f // 髋展关节下限
#define JOINT1_MAX 30.0f  // 髋展关节上限
#define JOINT2_MIN -90.0f // 髋转关节下限
#define JOINT2_MAX 40.0f  // 髋转关节上限
#define JOINT3_MIN 0.0f   // 膝关节下限
#define JOINT3_MAX 170.0f // 膝关节上限

/* 核对 Foot_mode_control.c 中全部关节的 ID、方向、零位和限位后置为 1。 */
#ifndef FOOT_JOINTS_CALIBRATED
#define FOOT_JOINTS_CALIBRATED 0
#endif

/*============================================================================
 * 三、站立姿态默认角度 (度)
 *============================================================================*/
#define STAND_J1 0.0f  // 髋展关节 (0=正中)
#define STAND_J2 35.0f // 髋转关节 (正值=前倾)
#define STAND_J3 70.0f // 膝关节 (正值=后弯)

/*============================================================================
 * 四、轮式姿态角度 (度) —— 身体降低，轮子着地
 *============================================================================*/
#define WHEEL_J2 35.0f  // 轮式髋转角（更大前倾，降低身体）
#define WHEEL_J3 70.0f // 轮式膝关节角（更大弯曲，降低身体）
#define STAND_TRANS_DUR 0.5f // 站立过渡时长 (s)

/*============================================================================
 * 五、步态参数
 *============================================================================*/

// 步态选择 (取消注释其中一个即可切换)
#define TEST_TROT
// #define TEST_WALK
// #define TEST_PRONK

// --- Trot 对角小跑 ---
#define TROT_CYCLE_TIME 0.6f
#define TROT_SWING_RATIO 0.45f
#define TROT_STEP_HEIGHT 0.04f // 抬腿高度 (m)，腿长0.2m，0.04合理
#define TROT_MAX_STEP_X 0.18f  // 需 >= step_x(0.12) + MAX_YAW_STEP(0.05) = 0.17，给偏航修正留空间
#define TROT_MAX_STEP_Y 0.07f
#define STEP_RATE_LIMIT 0.005f // max step length change per control tick (m)
/*
#define TROT_CMD_V_X      -0.005575f
#define TROT_CMD_V_Y       0.0025675f
#define TROT_CMD_V_YAW    -0.223f
*/
#define TROT_CMD_V_X 0.0f // 需保证 step_x(=V*T=0.12) < max_step_x(0.15)，留0.03给偏航修正
#define TROT_CMD_V_Y 0.5f
#define TROT_CMD_V_YAW 0.0f

// --- Walk 慢走 ---
#define WALK_CYCLE_TIME 1.2f
#define WALK_SWING_RATIO 0.25f
#define WALK_STEP_HEIGHT 0.03f
#define WALK_MAX_STEP_X 0.1f
#define WALK_MAX_STEP_Y 0.05f
#define WALK_CMD_V_X 0.0f
#define WALK_CMD_V_Y 0.3f
#define WALK_CMD_V_YAW 0.0f

// --- Pronk 四腿蹦跳 ---
#define PRONK_CYCLE_TIME 0.4f
#define PRONK_SWING_RATIO 0.40f
#define PRONK_STEP_HEIGHT 0.08f
#define PRONK_MAX_STEP_X 0.06f
#define PRONK_MAX_STEP_Y 0.03f
#define PRONK_CMD_V_X 0.04f //.8f
#define PRONK_CMD_V_Y 2.0f
#define PRONK_CMD_V_YAW 0.0f

// 根据步态选择映射为通用宏名（供代码中统一使用 CYCLE_TIME / SWING_RATIO 等）
#ifdef TEST_TROT
#define CYCLE_TIME TROT_CYCLE_TIME
#define SWING_RATIO TROT_SWING_RATIO
#define STEP_HEIGHT TROT_STEP_HEIGHT
#define MAX_STEP_X TROT_MAX_STEP_X
#define MAX_STEP_Y TROT_MAX_STEP_Y
#define CMD_V_X TROT_CMD_V_X
#define CMD_V_Y TROT_CMD_V_Y
#define CMD_V_YAW TROT_CMD_V_YAW
#endif

#ifdef TEST_WALK
#define CYCLE_TIME WALK_CYCLE_TIME
#define SWING_RATIO WALK_SWING_RATIO
#define STEP_HEIGHT WALK_STEP_HEIGHT
#define MAX_STEP_X WALK_MAX_STEP_X
#define MAX_STEP_Y WALK_MAX_STEP_Y
#define CMD_V_X WALK_CMD_V_X
#define CMD_V_Y WALK_CMD_V_Y
#define CMD_V_YAW WALK_CMD_V_YAW
#endif

#ifdef TEST_PRONK
#define CYCLE_TIME PRONK_CYCLE_TIME
#define SWING_RATIO PRONK_SWING_RATIO
#define STEP_HEIGHT PRONK_STEP_HEIGHT
#define MAX_STEP_X PRONK_MAX_STEP_X
#define MAX_STEP_Y PRONK_MAX_STEP_Y
#define CMD_V_X PRONK_CMD_V_X
#define CMD_V_Y PRONK_CMD_V_Y
#define CMD_V_YAW PRONK_CMD_V_YAW
#endif

/*============================================================================
 * 五、姿态 PID 控制参数
 *============================================================================*/

// --- 默认 / Trot 参数 ---
#define PD_KP_PITCH 0.008f         // 俯仰比例
#define PD_KD_PITCH 0.004f         // 俯仰微分
#define PD_KP_ROLL 0.006f          // 横滚比例
#define PD_KD_ROLL 0.005f          // 横滚微分
#define PD_MAX_CORRECTION 0.03f    // Z高度最大修正量 (m)
#define PD_KP_YAW 0.05f            // 偏航比例 (m/rad)
#define PD_KI_YAW 0.02f            // 偏航积分 (m/(rad·s))
#define PD_KD_YAW 0.02f            // 偏航微分 (m·s/rad)
#define PD_MAX_YAW_STEP 0.03f      // 偏航差步上限 (m)
#define PD_MAX_YAW_INTEGRAL 0.015f // 偏航积分上限 (防饱和)
#define PD_DZ_PITCH_RAD 0.017f     // 俯仰死区 (~1°)
#define PD_DZ_ROLL_RAD 0.017f      // 横滚死区 (~1°)
#define PD_DZ_YAW_RAD 0.035f       // 偏航死区 (~2°)

// --- Walk 参数 (更柔和的增益) ---
#define PD_WALK_KP_PITCH 0.0000005f
#define PD_WALK_KD_PITCH 0.00025f
#define PD_WALK_KP_ROLL 0.0000005f
#define PD_WALK_KD_ROLL 0.00025f
#define PD_WALK_MAX_CORRECTION 0.03f
#define PD_WALK_KP_YAW 0.15f           // 偏航比例 (m/rad)
#define PD_WALK_KI_YAW 0.20f           // 偏航积分 (m/(rad·s))，加速消除稳态偏差
#define PD_WALK_KD_YAW 0.02f           // 偏航微分 (m·s/rad)
#define PD_WALK_MAX_YAW_STEP 0.05f     // 偏航差步上限 (m)
#define PD_WALK_MAX_YAW_INTEGRAL 0.80f // 偏航积分上限，给足够空间积累以消除稳态偏差
#define PD_WALK_DZ_PITCH_RAD 0.017f
#define PD_WALK_DZ_ROLL_RAD 0.017f
#define PD_WALK_DZ_YAW_RAD 0.017f // 偏航死区 (~1°)

// PONK参数
//  --- Walk 参数 (更柔和的增益) ---
#define PD_PONK_KP_PITCH 0.0000005f
#define PD_PONK_KD_PITCH 0.00025f
#define PD_PONK_KP_ROLL 0.0000005f
#define PD_PONK_KD_ROLL 0.00025f
#define PD_PONK_MAX_CORRECTION 0.03f
#define PD_PONK_KP_YAW 0.15f           // 偏航比例 (m/rad)
#define PD_PONK_KI_YAW 0.20f           // 偏航积分 (m/(rad·s))，加速消除稳态偏差
#define PD_PONK_KD_YAW 0.02f           // 偏航微分 (m·s/rad)
#define PD_PONK_MAX_YAW_STEP 0.05f     // 偏航差步上限 (m)
#define PD_PONK_MAX_YAW_INTEGRAL 0.80f // 偏航积分上限，给足够空间积累以消除稳态偏差
#define PD_PONK_DZ_PITCH_RAD 0.017f
#define PD_PONK_DZ_ROLL_RAD 0.017f
#define PD_PONK_DZ_YAW_RAD 0.017f // 偏航死区 (~1°)
/*============================================================================
 * 六、贝塞尔轨迹参数
 *============================================================================*/
#define MAX_TRAJECTORY_POINTS 100         // 离线轨迹最大离散点数
#define TIME_EPSILON 1e-4f                // 时间极小值 (防除零)
#define CONFIG_DEFAULT_LIFT_HEIGHT 0.055f // 默认抬腿高度 (m)

/*============================================================================
 * 七、支撑相参数
 *============================================================================*/
#define STANCE_Z_COMPENSATION 0.00f // 支撑相Z轴线性补偿量 (m)

/*============================================================================
 * 八、IK / 雅可比求解器参数
 *============================================================================*/
#define EPSILON 1e-6f                    // 通用数值极小值
#define JACOBIAN_EPSILON 1e-6f           // 雅可比数值稳定
#define JACOBIAN_SINGULAR_FULL_VAL 1e-4f // 完全奇异阈值
#define JACOBIAN_SINGULAR_NEAR_VAL 1e-3f // 接近奇异阈值
#define IK_JACOBIAN_ITER_MAX 30          // 最大迭代次数
#define IK_JACOBIAN_POS_TOL 1e-4f        // 位置收敛容差 (0.1mm)
#define IK_JACOBIAN_STEP_LARGE 0.2f      // 大步长
#define IK_JACOBIAN_STEP_SMALL 0.05f     // 小步长
#define IK_JACOBIAN_ERROR_THRESH 1e-3f   // 误差切换阈值 (1mm)

/*============================================================================
 * 九、跳跃控制器参数
 *============================================================================*/

// 电机爆发参数（跳跃时临时切换）
#define JUMP_MOTOR_VELOCITY 20.0f // 跳跃时电机速度 (rad/s)，需 > 角度变化量/蹬地时长
#define JUMP_MOTOR_TORQUE 80.0f   // 跳跃时电机最大扭矩 (N·m)，需克服自重 + 提供加速度

// 阶段时长 (秒)
#define JUMP_CROUCH_DUR 0.30   // 下蹲蓄力
#define JUMP_PUSH_OFF_DUR 0.15 // 蹬地腾空（80°/0.15s≈9.3rad/s，20rad/s有余量）
#define JUMP_AERIAL_DUR 0.35   // 空中调整
#define JUMP_LANDING_DUR 0.25  // 落地缓冲
#define JUMP_RECOVER_DUR 0.30  // 恢复站立

// 跳跃各阶段关节角度目标 (度，取绝对值；左右腿自动取反)
// 前腿（引导方向、先落地缓冲）
#define JUMP_CROUCH_J2_F 65.0f   // 前腿下蹲髋转角（深前倾→重心前移）
#define JUMP_CROUCH_J3_F 130.0f  // 前腿下蹲膝角（深蹲蓄力）
#define JUMP_PUSH_J2_F 25.0f     // 前腿蹬地髋转角
#define JUMP_PUSH_J3_F 50.0f     // 前腿蹬地膝角
#define JUMP_AERIAL_J2_F 50.0f   // 前腿空中髋转角（前伸准备落地）
#define JUMP_AERIAL_J3_F 90.0f   // 前腿空中膝角
#define JUMP_LANDING_J2_F 65.0f  // 前腿落地髋转角（深蹲吸震）
#define JUMP_LANDING_J3_F 135.0f // 前腿落地膝角

// 后腿（主发力蹬地、推动前冲）
#define JUMP_CROUCH_J2_R 30.0f   // 后腿下蹲髋转角（浅蹲→保持后高）
#define JUMP_CROUCH_J3_R 75.0f   // 后腿下蹲膝角（蓄力待蹬）
#define JUMP_PUSH_J2_R 0.0f      // 后腿蹬地髋转角（近垂直→向前上方发力）
#define JUMP_PUSH_J3_R 5.0f      // 后腿蹬地膝角（几乎伸直→爆发）
#define JUMP_AERIAL_J2_R 45.0f   // 后腿空中髋转角
#define JUMP_AERIAL_J3_R 95.0f   // 后腿空中膝角
#define JUMP_LANDING_J2_R 55.0f  // 后腿落地髋转角
#define JUMP_LANDING_J3_R 125.0f // 后腿落地膝角

/*============================================================================
 * 十、遥控参数
 *============================================================================*/
#define RC_MAX_VEL_X 0.3f        // 遥控最大前后速度 (m/s)
#define RC_MAX_VEL_Y 0.2f        // 遥控最大左右速度 (m/s)
#define RC_MAX_VEL_YAW 0.5f      // 遥控最大转向速度 (rad/s)
#define RC_SPEED_SCALE_STEP 0.2f // 加速/减速每次缩放步长
#define RC_SPEED_SCALE_MIN 0.1f  // 速度缩放下限
#define RC_SPEED_SCALE_MAX 3.0f  // 速度缩放上限
#define PITCH_ADJUST_STEP 0.035f // 俯仰角每次按键调整量 (rad, ≈2°)
#define PITCH_ADJUST_MAX 0.105f  // 俯仰角手动调整上限 (rad, ≈6°，共3次按键)

/*============================================================================
 * 十一、折叠姿态参数
 *============================================================================*/
#define FOLD_J1 30.0f      // 折叠时髋展外翻角度 (度)
#define FOLD_J2 80.0f      // 折叠时髋转角度 (度)
#define FOLD_J3 150.0f     // 折叠时膝关节角度 (度)
#define FOLD_DURATION 1.0f // 折叠动画时长 (秒)

/*============================================================================
 * 十二、轮式模式参数
 * 说明：轮式相关的全部常量集中在本节，Wheel_mode_contrl.h 不再重复定义，
 *       避免同一物理量存在两个值（历史上 WHEEL_RADIUS 有过 0.029f / 0.03f 两份）。
 *============================================================================*/
#define WHEEL_COUNT 4          // 轮子数量
#define WHEEL_RADIUS 0.03f     // 轮子半径 (m)，如实测为 0.029m 请只改这一处
#define WHEEL_BASE_HALF 0.078f // 左右轮距的一半 (m)，取FOOT_OFFSET_Y，与FOOT_OFFSET_Y保持一致
#define WHEEL_MAX_SPEED 10.0f  // 轮子最大转速 (rad/s)，差速解算后按此限幅
/* 核对四个电机的 ID、反馈 ID、正转方向和有效轮径后再允许使能。 */
#ifndef WHEEL_MOTORS_CONFIGURED
#define WHEEL_MOTORS_CONFIGURED 0
#endif

/* 新固件的使能和失能使用原始 ID，使用速度模式 ID 的固件填 0x200。 */
#define WHEEL_DM_SERVICE_OFFSET 0x000u

/* 占位映射。正转方向为轮子推动机器人前进时的电机旋转方向。 */
#define WHEEL_CAN_ID_RF 0x01u
#define WHEEL_CAN_ID_LF 0x02u
#define WHEEL_CAN_ID_RB 0x03u
#define WHEEL_CAN_ID_LB 0x04u
#define WHEEL_FEEDBACK_ID_RF 0x11u
#define WHEEL_FEEDBACK_ID_LF 0x12u
#define WHEEL_FEEDBACK_ID_RB 0x13u
#define WHEEL_FEEDBACK_ID_LB 0x14u
#define WHEEL_DIRECTION_RF 1
#define WHEEL_DIRECTION_LF 1
#define WHEEL_DIRECTION_RB 1
#define WHEEL_DIRECTION_LB 1

/*============================================================================
 * 十三、调试打印开关
 * 打印走的是阻塞式串口输出(fputc 死等 TC)，在 upctrl_task 内会占满控制周期，
 * 上板默认全部关闭。需要看日志时把 ROBOT_DEBUG_PRINT 置 1 重新编译即可。
 *============================================================================*/
#define ROBOT_DEBUG_PRINT 0 // 调试打印总开关：0=关闭(上板默认)，1=打开

#endif // ROBOT_CONFIG_H
