#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "include_head.h"

// ======================== 机器人主模态枚举 ========================
typedef enum
{
    ROBOT_MODE_IDLE = 0,   // 待机（所有执行器断电）
    ROBOT_MODE_QUADRUPED,  // 四足模式（走路/小跑/奔跑）
    ROBOT_MODE_WHEELED,    // 轮式模式（轮子驱动）
    ROBOT_MODE_FLIGHT,     // 飞行模式（旋翼驱动）
    ROBOT_MODE_TRANSITION, // 模态切换过渡状态
    ROBOT_MODE_ERROR       // 全局错误状态
} RobotMode_t;

// ======================== 四足子模式枚举 ========================
typedef enum
{
    QUAD_MODE_STAND = 0, // 站立
    QUAD_MODE_WALK,      // 行走
    QUAD_MODE_TROT,      // 对角小跑
    QUAD_MODE_CRAWL,     // 爬行
    QUAD_MODE_PRONK
} QuadSubMode_t;

// ======================== 轮式子模式枚举 ========================
typedef enum
{
    WHEEL_MODE_STOP = 0,   // 停止
    WHEEL_MODE_FORWARD,    // 前进
    WHEEL_MODE_BACKWARD,   // 后退
    WHEEL_MODE_TURN_LEFT,  // 左转
    WHEEL_MODE_TURN_RIGHT, // 右转
    WHEEL_MODE_SPIN        // 原地旋转
} WheelSubMode_t;

// ======================== 飞行子模式枚举 ========================
typedef enum
{
    FLIGHT_MODE_HOVER = 0, // 悬停
    FLIGHT_MODE_TAKEOFF,   // 起飞
    FLIGHT_MODE_LANDING,   // 降落
    FLIGHT_MODE_FORWARD,   // 前飞
    FLIGHT_MODE_BACKWARD,  // 后飞
    FLIGHT_MODE_LEFT,      // 左飞
    FLIGHT_MODE_RIGHT,     // 右飞
    FLIGHT_MODE_CLIMB,     // 爬升
    FLIGHT_MODE_DESCEND    // 下降
} FlightSubMode_t;

// ======================== 全局机器人状态（三模态版） ========================
typedef struct
{
    // -------------------------- 全局通用状态 --------------------------
    RobotMode_t mode;          // 当前主模态
    RobotMode_t target_mode;   // 目标模态（切换时用）
    RobotMode_t last_mode;     // 上一个模态
    float transition_progress; // 模态切换进度（0~1）
    uint8_t is_transitioning;  // 是否正在切换模态

    // 身体姿态（所有模态通用），三个角均为弧度，由IMU融合结果直接写入，不再换算
    float body_height; // 身体离地高度（m）
    float body_pitch;  // 俯仰角（rad，向右倾斜是正
    float body_roll;   // 横滚角（rad，向下低头是正）
    float body_yaw;    // 偏航角（rad，向左转是正）

    // 全局速度指令（所有模态通用）
    float cmd_vel_x;   // 前进速度（m/s）
    float cmd_vel_y;   // 侧移速度（m/s）
    float cmd_vel_yaw; // 转向速度（rad/s）
    float cmd_vel_z;   // 垂直速度（m/s，飞行模式用）

    // 全局仿真时间
    float sim_time_s; // 当前仿真时间（s）
    float dt_s;       // 控制周期（s，默认0.001s）

    // -------------------------- 四足模式专属状态 --------------------------
    QuadSubMode_t quad_sub_mode; // 四足子模式
    float step_length;           // 步长（m）
    float swing_height;          // 抬腿高度（m）

    // -------------------------- 轮式模式专属状态 --------------------------
    WheelSubMode_t wheel_sub_mode; // 轮式子模式
    float wheel_speed[4];          // 四个轮子的转速（rad/s）
    float wheel_steer_angle[2];    // 转向轮角度（°，前轮转向）
    float wheel_max_speed;         // 轮子最大转速（rad/s）

    // -------------------------- 飞行模式专属状态 --------------------------
    FlightSubMode_t flight_sub_mode; // 飞行子模式
    float flight_height;             // 飞行高度（m）
    float target_flight_height;      // 目标飞行高度（m）
    float throttle[4];               // 四个旋翼油门（0~1）
    float max_throttle;              // 最大油门
    float min_throttle;              // 最小油门

    // -------------------------- 传感器状态 --------------------------
    float imu_acc[3];          // IMU加速度（m/s?）
    float imu_gyro[3];         // IMU角速度（rad/s）
    float foot_force[4];       // 足端压力（N）
    uint8_t wheel_contact[4];  // 轮子接触地面标志
    uint8_t is_ground_contact; // 是否接触地面
} RobotGlobalState_t;

// 全局状态变量
extern RobotGlobalState_t g_robot_state;

// 初始化全局状态
void robot_state_init(void);

// 重置全局状态到安全值
void robot_state_reset(void);

// 模态切换请求（安全检查+启动切换）
int robot_mode_switch_request(RobotMode_t target_mode);

// 检查模态切换是否允许
int robot_mode_switch_allowed(RobotMode_t current_mode, RobotMode_t target_mode);

#endif // ROBOT_STATE_H
