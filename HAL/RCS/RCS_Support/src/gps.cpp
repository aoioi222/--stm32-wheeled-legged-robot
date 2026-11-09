/**
 * @filename gps.cpp
 * @brief 硬件无关的全场定位解算算法
 * @author 移植自RCS14队长陈志伟的论文
 * @date  2025-06-04
 * ----------------------------------------------------------------------
**/
#include "gps.h"
#include <cmath>

#include <FreeRTOS.h>
#include <task.h>

/**
 * @brief 全场定位解算类
 *
 * @param TRANSFORM_X 单位转换参数。 X方向的编码轮的数据(无单位或单位不是mm) * TRANSFORM_X = X方向编码轮实际走过的距离(mm)。可以是负数，用于将左手坐标系转为右手坐标系。
 * @param TRANSFORM_Y 单位转换参数。 Y方向的编码轮的数据(无单位或单位不是mm) * TRANSFORM_Y = Y方向编码轮实际走过的距离(mm)。可以是负数，用于将左手坐标系转为右手坐标系。
 * 
 * @param ANGLE_DX 坐标系旋转参数。 描述的是全场定位坐标系和车体坐标系之间的夹角，单位为rad
 * @param ANGLE_DY 坐标系旋转参数。 请保持和ANGLE_DX的一致。
 *
 * @param TRANSFORM_AX 坐标系平移参数。 描述的是全场定位坐标系和车体坐标系之间的X平移，单位为mm
 * @param TRANSFORM_AY 坐标系平移参数。 描述的是全场定位坐标系和车体坐标系之间的Y平移，单位为mm
 *
 * -----------------------------------------------------------------------------------------------------
 * @note 前置知识1：坐标系
 * 1. 定位坐标系：正交全向轮做延长线，交点即为全场定位坐标系的原点，延长线一条为X轴一条为Y轴。
 * 2. 底盘坐标系：方向由用户自行定义，原点为车体的几何中心点。
 * 3. 场地坐标系：以比赛场地作为坐标系，机器人出发区为原点，XY轴平行于场地边缘。
 * 4. 三个坐标系都是右手坐标系。也即X轴逆时针转90度到Y轴、逆时针旋转视为角度增加。
 * 5. 如何描述两个坐标系之间的位置关系？使用xy描述平动，θ描述转动。换言之，xy坐标系其实是三维的。 
 * ------------------------------------------------------------------------------------------------------
 * @note 前置知识2：全场定位的概念
 * 1. 全场定位模块在底盘上固定安装，因此定位坐标系和底盘坐标系之间的距离(Δx,Δy,Δθ)是固定值。
 * 2. 机器人在场地上可以以(vx,vy,vθ)任意移动，所以底盘坐标系和场地坐标系的距离不是固定值。
 * 3. 所谓全场定位，实际上是获取机器人相较于出发区的绝对位置(x,y,θ)，也可以表达为"底盘坐标系在场地坐标系下的坐标"。
 * -------------------------------------------------------------------------------------------------------
 * @note 前置知识3：全场定位算法的机理和优缺点
 * 1. 总所周知，速度积分可以得到距离，因此我们可以利用这种方式估计(x,y,θ)，这就是全场定位算法的本质。
 * 2. 利用积分估计的传感器，称为惯性传感器。惯性传感器有个严重的缺点，那就是越久误差越大。
 * 3. 为什么呢？因为电子设备是离散系统。因此我们只能做离散意义上的积分，也就是求和。
 * 4. 求和出来的结果肯定与积分是有误差的，而且因为加和作用，误差逐步累计，进而越来越大。
 *
 * 5. 正因如此，我们一般不会单独使用全场定位，而是全场定位+激光，二者其实优缺点互补：
 * 6. 全场定位是惯性传感器，测量方差很小，但随着时间推移误差会逐渐增大。
 * 7. 激光测距/激光雷达是ToF(Time of Flight)传感器，测量方差较大，但总体误差不随时间推移而改变。
 *
 * 8. 对于固定方向的激光测距，一般的策略是定期校准。比如车子跑到了某个点位，刚好停下来的时候，就可以趁机用激光修一下坐标。
 * 9. 对于反复扫描的激光雷达，一般的策略是融合滤波，因为现在SLAM算法已经很发达了，能在任意时刻估计坐标，把SLAM得到的坐标和全场定位融合一下，就优缺点互补了。
 * ---------------------------------------------------------------------------------------------------------
 * @note 使用队里的代码把全场定位算法用起来
 * 全场定位总共有五个参数需要给定，其实可以按功能分成两大类：
 *
 * 1. 坐标系标准化参数(TRANSFORM_X/TRANSFORM_Y)：编码轮获取到的是角度或者脉冲数，但我们需要的是mm，因此需要做单位转换。
 *                                               除此之外，有时编码器的坐标系是左手的，也需要利用这两个参数，先把他扭成右手。
 *
 * 2. 坐标系转换参数(ANGLE_DX/TRANSFORM_AX/TRANSFORM_AY): 可以在机器人的任意位置安装全场定位，不一定要在车体中心，或是与车体平行。
 *                                                        这三个参数就是用来描述全场定位在车上的位置的，功能是把任意安装的全场定位重新投影到车体中心+车体平行。
 * ---------------------------------------------------------------------------------------------------------
 * @warning 注意事项
 * 1. 请务必认真对待ANGLE_DX/TRANSFORM_AX/TRANSFORM_AY三个参数，认真将全场定位坐标系和场地坐标系对齐，不能因为刚好差90度就不管了。
 *    因为只有坐标系对齐了，自动跑点才能构成闭环，否则PID就会变成开环控制，引起电机疯转。
 * ---------------------------------------------------------------------------------------------------------
**/
gps::gps(double TRANSFORM_X, double TRANSFORM_Y, double ANGLE_DX, double ANGLE_DY,
         double TRANSFORM_AX, double TRANSFORM_AY, uint8_t CAC_FREQ)
    : transform_x(TRANSFORM_X), transform_y(TRANSFORM_Y),
      angle_dx(ANGLE_DX), angle_dy(ANGLE_DY),
      transform_ax(TRANSFORM_AX), transform_ay(TRANSFORM_AY),
      cac_freq(CAC_FREQ),
      last_org_x(0.0), last_org_y(0.0), last_zangle(0.0),
      raw_gps_x(0.0), raw_gps_y(0.0),
      cycle_z(0)
{
   
}

/**
 * @brief 给定机器人获取定位坐标系下(x,y,θ)的函数，然后在后台启动全场定位解算
 *
 * @param x_encoder,y_encoder,z_encoder 这三个函数指针获取的都是距离，不要填成获取速度了。
**/
rcs_err_t gps::init(port_fptr x_encoder, port_fptr y_encoder, port_fptr z_gyro)
{
    this->get_encoder_x = x_encoder;
    this->get_encoder_y = y_encoder;
    this->get_gyro_z = z_gyro;
	if (xTaskCreate(gps::gps_task,NULL,128,(void*)this,RCS_GPS_TASK_PRIO,NULL)!=pdPASS) {
		return RCS_FAIL;
	}
    return RCS_OK;
}

/**
 * @brief 获取全场定位算法解算出的x坐标，单位mm
 * @note 这里的坐标指的是： [机器人当前位置] 相较于 [机器人出发位置] 的 [(x,y,θ)绝对距离]
 *
 * @warning 请确保本函数在setup函数后调用。因为坐标是在后台计算的，而不是由本函数计算的。
*           本函数只负责返回计算结果，仅此而已。
**/
double gps::get_pos_x()
{
    return gps_x;
}

/**
 * @brief 获取全场定位算法解算出的y坐标，单位mm
 * @note 这里的坐标指的是： [机器人当前位置] 相较于 [机器人出发位置] 的 [(x,y,θ)绝对距离]
 *
 * @warning 请确保本函数在setup函数后调用。因为坐标是在后台计算的，而不是由本函数计算的。
*           本函数只负责返回计算结果，仅此而已。
**/
double gps::get_pos_y()
{
    return gps_y;
}

/**
 * @brief 获取全场定位算法解算出的z坐标，单位rad
 * @note 这里的坐标指的是： [机器人当前位置] 相较于 [机器人出发位置] 的 [(x,y,θ)绝对距离]
 *
 * @warning 请确保本函数在setup函数后调用。因为坐标是在后台计算的，而不是由本函数计算的。
*           本函数只负责返回计算结果，仅此而已。
**/
double gps::get_pos_z()
{
    return gps_z;
}

/**
 * @brief 对全场定位解算出来的X坐标进行修正
 * @param x 使用ToF传感器测量得到的坐标x。这里的坐标指的是： [机器人当前位置] 相较于 [机器人出发位置] 的 [(x,y,θ)绝对距离]
**/
void gps::set_pos_x(double x)
{
	gps_x = x;
}

/**
 * @brief 对全场定位解算出来的Y坐标进行修正
 * @param y 使用ToF传感器测量得到的坐标x。这里的坐标指的是： [机器人当前位置] 相较于 [机器人出发位置] 的 [(x,y,θ)绝对距离]
**/
void gps::set_pos_y(double y)
{
	gps_y = y;
}

/**
 * @brief 全场定位算法进行坐标解算的后台任务函数
 * @note RTOS会在xTaskCreate函数被调用后，自动在合适的时机调用任务函数，无需再由用户手动调用
**/
void gps::gps_task(void *args)
{
	gps* handle = (gps*)args;
	
	while(1) {
		if (!handle->get_encoder_x || !handle->get_encoder_y || !handle->get_gyro_z)
			return;

		double raw_z = handle->get_gyro_z();             // 获取偏航角（角度）
		double rad_zangle = raw_z * DEG2RAD;     // 转弧度

		// 环绕修正
		if (handle->last_zangle >= 160.0 && raw_z <= -160.0) handle->cycle_z++;
		if (handle->last_zangle <= -160.0 && raw_z >= 160.0) handle->cycle_z--;

		double gps_sum_zangle = handle->cycle_z * 360.0 + raw_z;

		// 获取编码器原始位移
		double gps_origin_x = handle->get_encoder_x();
		double gps_origin_y = handle->get_encoder_y();

		// 计算增量（乘轮径比例系数）
		double delta_x = (gps_origin_x - handle->last_org_x) * handle->transform_x;
		double delta_y = (gps_origin_y - handle->last_org_y) * handle->transform_y;

		// 累加世界坐标系下的位移（原点为参考）
		handle->raw_gps_x += delta_x * std::cos(rad_zangle) - delta_y * std::sin(rad_zangle);
		handle->raw_gps_y += delta_x * std::sin(rad_zangle) + delta_y * std::cos(rad_zangle);

		// 保存当前值作为下次计算的参考
		handle->last_org_x = gps_origin_x;
		handle->last_org_y = gps_origin_y;
		handle->last_zangle = raw_z;

		// 定位仪到车中心的变换
		handle->gps_x = -handle->raw_gps_x * std::cos(handle->angle_dx) - handle->raw_gps_y * std::sin(handle->angle_dy) + std::sin(rad_zangle) * handle->transform_ax;
		handle->gps_y = handle->raw_gps_x * std::sin(handle->angle_dx) - handle->raw_gps_y * std::cos(handle->angle_dy) + handle->transform_ay * (1 - std::cos(rad_zangle));
		handle->gps_x /= 10.0;
		handle->gps_y /= 10.0;
		handle->gps_z = rad_zangle;
		
		// 解算周期
		vTaskDelay(pdMS_TO_TICKS(handle->cac_freq));
	}
}


