/**
 * @file kin_diff.h
 * @author 陈煜楷(CYK-Dot@github.com)
 * @brief 差动机构(例如差动机械臂、差动舵轮)的解算库
 * @version 0.1
 * @date 2025-06-07
 * 
 * @copyright Copyright 陈煜楷 (c) 2025
 * 
 */
#pragma once

/**
 * @brief 差动机构(例如差动机械臂、差动舵轮)的解算库
 * 
 * @tparam TFLOAT 使用什么浮点类型
 */
template<typename TFLOAT> class diff
{
	private:
		TFLOAT upRadius;
		TFLOAT downRadius;
		TFLOAT outRadius;
	
	public:
		/**
		 * @brief 解算对象的构造函数
		 * 
		 * @param Ru 差动舵轮上锥齿轮的半径/差动机械臂输入自由度1锥齿轮的半径
		 * @param Rd 差动舵轮下锥齿轮的半径/差动机械臂输入自由度2锥齿轮的半径
		 * @param Ro 差动舵轮输出锥齿轮的半径/差动机械臂输出自由度锥齿轮的半径
		 */
		diff(TFLOAT Ru, TFLOAT Rd, TFLOAT Ro)
        : upRadius(Ru), downRadius(Rd), outRadius(Ro) {}
		
		/**
		 * @brief 正运动学：获取输出轴公转的速度/力矩
		 * 
		 * @param up 上锥齿轮的速度/力矩
		 * @param down 下锥齿轮的速度/力矩
		 * @return TFLOAT 输出轴公转的速度/力矩
		 */
		TFLOAT KinServo(TFLOAT up,TFLOAT down)
		{
			TFLOAT res = upRadius/(upRadius + downRadius) * up + 
			             downRadius/(upRadius + downRadius) * down;
			return res;
		}
		/**
		 * @brief 正运动学：获取输出轴自转的速度/力矩
		 * 
		 * @param up 上锥齿轮的速度/力矩
		 * @param down 下锥齿轮的速度/力矩
		 * @return TFLOAT 输出轴自转的速度/力矩
		 */
		TFLOAT KinDrive(TFLOAT up,TFLOAT down)
		{
			TFLOAT temp = upRadius*downRadius/((outRadius)*(upRadius + downRadius));
			return (-temp*up +temp*down);
		}
		/**
		 * @brief 逆运动学：获取上锥齿轮的速度/力矩
		 * 
		 * @param servo 输出轴公转的速度/力矩
		 * @param drive 输出轴自转的速度/力矩
		 * @return TFLOAT 
		 */
		TFLOAT InvUp(TFLOAT servo,TFLOAT drive)
		{
			return servo - outRadius/upRadius * drive;
		}
		
		/**
		 * @brief 逆运动学：获取下锥齿轮的速度/力矩
		 * 
		 * @param servo 输出轴公转的速度/力矩
		 * @param drive 输出轴自转的速度/力矩
		 * @return TFLOAT 
		 */
		TFLOAT InvDown(TFLOAT servo,TFLOAT drive)
		{
			return servo + outRadius/downRadius * drive;
		}
};