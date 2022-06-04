/*
 * robotApi.h
 *
 *  Created on: Nov 24, 2020
 *      Author: liuchongyang
 */
#ifndef ROBOT_H_
#define ROBOT_H_

#include<deque>
#include<memory>
#include<stdio.h>


#include "../eigen/Eigen/Eigen"
#include "struct_define.h"

using namespace Eigen;

#define rad2deg(r) ((r)*180.0/M_PI)
#define deg2rad(d) ((d)*M_PI/180.0)

class Axis;

class Robot
{
public:
	Robot();
	virtual ~Robot(){};
	/**
	* @brief 设置DH参数
	*/
	void set_DH_param(DH_param param_set);
	/**
	* @brief 获取DH参数
	*/
	DH_param get_DH_param();
	/**
	* @brief 设置笛卡尔参数
	*/
	void set_decare_param(Decare_Para param_set);
	/**
	* @brief 获取笛卡尔参数
	*/
	Decare_Para get_decare_param();
	/**
	* @brief 设置电机参数
	*/
	void set_motor_param(Motor_Param param_set);
	/**
	* @brief 获取电机参数
	*/
	Motor_Param get_motor_param();
	/**
	* @brief 设置运动队列
	*/
	void set_angle_deque(std::deque<double>& deque);
	/**
	* @brief 获取运动队列
	*/
	std::deque<double>& get_angle_deque();

	/**
	* @brief 判断g_axis是否为空
	*/
	bool g_axis_is_NULL();
	/**
	* @brief 使能
	*/
	void power_on();
	/**
	* @brief 使能关闭
	*/
	void power_off();
	/**
	* @brief 获取实际位置角度值
	*/
	double get_actual_position(int axis);
	/**
	* @brief 设置目标位置
	*/
	void set_target_position(int axis,double targetPosition);

	/**
	* @brief 获取使能状态
	*/
	bool get_power_on_status();

	/**
	* @brief 获取实际转矩
	*/
	int get_actual_torque(int axis);

	/**
	* @brief 获取状态字
	*/
	int get_status_word(int axis);

	/**
	* @brief 获取实际速度,速度单位取决于伺服输出的单位
	*/
	int get_actual_velocity(int axis);
	/**
	* @brief 设置目标转矩
	*/
	void set_target_torque(int axis,int targetTorque);
	/**
	* @brief 通过sdo向目标地址写值
	* @param axis 轴编号，从0开始
	* @param index 地址主索引
	* @param subindex 地址子索引
	* @param value 要写入的值
	* @param size 要写入的值的类型的大小
	*/
	int set_SDO(unsigned int axis, unsigned int index, unsigned int subindex, unsigned char* value, unsigned int size);
	/**
	* @brief 通过sdo获取目标地址的值
	* @param axis 轴编号，从0开始
	* @param index 地址主索引
	* @param subindex 地址子索引
	* @param value 读取到的值
	* @param size 要读取的值的类型的大小
	*/
	int get_SDO(unsigned int axis, unsigned int index, unsigned int subindex, unsigned char* value, unsigned int size);

	/**
	* @brief 添加pdo对象
	* @param index pdo对象主索引
	* @param type pdo数据类型,详见 pdo_object_type
	*/
	static void add_pdo(unsigned int index,	pdo_object_type type);

	/**
	* @brief 通过pdo向目标地址写值
	* @param index pdo对象主索引
	* @param axis 轴编号，从0开始
	* @param value 要写入的值,详见 Pdo_value
	*/
	void set_pdo_value(unsigned int index,int axis,Pdo_value value);

	/**
	* @brief 通过pdo获取目标地址的值
	* @param index pdo对象主索引
	* @param axis 轴编号，从0开始
	* @return 读取到的值,详见 Pdo_value
	*/
	Pdo_value get_pdo_value(unsigned int index,int axis);

	/**
	* @brief 主站调用运行函数
	*/
	void register_cycle_run();

	static int call_cycle_run();
	/**
	* @brief 循环运行函数
	*/
	virtual void cycle_run();
	
	/**
	* @brief 机器人正解,传入的关节值单位是角度
	*/
	virtual void calc_forward_kin(const VectorXd& posACS,MatrixXd& transMatrix) = 0;
	/**
	* @brief 机器人逆解,传出的关节值单位是角度
	*/
	virtual void calc_inverse_kin(const MatrixXd& transMatrix,const VectorXd& posLast, VectorXd& posACS) = 0;
	/**
	* @brief 机器人关节运动插补
	*/
	virtual void move_joint_interp(const VectorXd &targetPoint,
			const VectorXd &originPoint, const VectorXd &velCurrent, const VectorXd &accCurrent, double Ts, double velPerc,
			double accPerc, double decPerc, double jerkPerc,std::deque<double> &nAglSeqPtr) = 0;
	/**
	* @brief 机器人直线运动正解
	*/
	virtual void move_line_interp(const VectorXd &targetPoint,
			const VectorXd &originPoint, const VectorXd &originACS, double velCurrent, double accCurrent,
			double Ts, double maxVelL, double maxAccL, double maxDecelL,
			double maxJerk, std::deque<double> &nAglSeqPtr) = 0;

protected:
	DH_param dh; ///DH参数对象
	Decare_Para decare;  ///笛卡尔参数对象
	Motor_Param motor_param;  ///电机参数对象
	std::deque<double> angle_deque;  ///机器人运动队列
private:
	Axis* g_axis;
	static Robot* user_rbt_prt;
};


#endif /* ROBOT_H_ */
