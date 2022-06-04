#ifndef GENERAL_6S_H_
#define GENERAL_6S_H_

#include "project_base/robot.h"

class General_6S : public Robot
{
public:
	General_6S() : Robot(){};
	/**
	 * @brief 机器人正解,传入的关节值单位是角度
	 * @param posACS 各关节角度值
	 * @param transMatrix 正解得到的转移矩阵
	 */
	virtual void calc_forward_kin(const VectorXd &posACS, MatrixXd &transMatrix);
	/**
	 * @brief 机器人逆解,传出的关节值单位是角度
	 * @param transMatrix 转移矩阵
	 * @param posLast 逆解参考位置,单位是角度
	 * @param posACS 逆解得到的关节值,单位是角度
	 */
	virtual void calc_inverse_kin(const MatrixXd &transMatrix, const VectorXd &posLast, VectorXd &posACS);
	/**
	 * @brief 机器人关节运动插补
	 * @param targetPoint目标点的关节位置
	 * @param originPoint 起始点的关节位置
	 * @param velCurrent 起始点的速度
	 * @param accCurrent 起始点的加速度
	 * @param Ts 插补周期
	 * @param velPerc 速度百分比
	 * @param accPerc 加速度百分比
	 * @param decPerc 减速度百分比
	 * @param jerkPerc 雅可比速度百分比
	 * @param nAglSeqPtr 插补得到的队列
	 */
	virtual void move_joint_interp(const VectorXd &targetPoint,
								   const VectorXd &originPoint, const VectorXd &velCurrent, const VectorXd &accCurrent, double Ts, double velPerc,
								   double accPerc, double decPerc, double jerkPerc, std::deque<double> &nAglSeqPtr);
	/**
	 * @brief 机器人直线运动正解
	 * @param targetPoint 目标点的直角位置
	 * @param originPoint 起始点的直角位置
	 * @param originACS 起始点的关节位置
	 * @param velCurrent 起始点的速度
	 * @param accCurrent 起始点的加速度
	 * @param Ts 插补周期
	 * @param maxVelL 最大直线速度
	 * @param maxAccL 最大直线加速度
	 * @param maxDecelL 最大直线减速度
	 * @param maxJerk 最大雅可比速度
	 * @param maxJerk nAglSeqPtr 插补得到的队列
	 */
	virtual void move_line_interp(const VectorXd &targetPoint,
								  const VectorXd &originPoint, const VectorXd &originACS, double velCurrent, double accCurrent,
								  double Ts, double maxVelL, double maxAccL, double maxDecelL,
								  double maxJerk, std::deque<double> &nAglSeqPtr);
	/**
	 * @brief 转移矩阵转直角坐标
	 * @param m 转移矩阵
	 * @return 返回六维直角坐标值
	 */
	VectorXd tr_2_MCS(MatrixXd m);
	/**
	 * @brief 直角坐标转转移矩阵
	 */
	MatrixXd rpy_2_tr(VectorXd posMCS);

private:
	/**
	 * @brief 位姿转转移矩阵
	 */
	MatrixXd rpy_2_r(Vector3d rpy);
	/**
	 * @brief 直角坐标转转移矩阵
	 */
	// MatrixXd rpy_2_tr(VectorXd posMCS);
	/**
	 * @brief 转移矩阵转位姿
	 */
	Vector3d tr_2_rpy(MatrixXd m);

	/**
	 * @brief 计算五次多项式插补系数
	 * @param q0 初始位置
	 * @param qf 目标位置
	 * @param q0 初始速度
	 * @param qf 目标速度
	 * @param ddq0 初始加速度
	 * @param ddqf 目标加速度
	 * @param tf 插补周期
	 * @param Coe 五次多项式结构体参数
	 */
	void get_Quintic_Coe(double q0, double qf, double dq0, double dqf,
						 double ddq0, double ddqf, double tf, Quintic_Coe &Coe);
	/**
	 * @brief 取符号
	 * @param d 需要取符号的参数
	 * @return d 大于0时返回1，d小于0时返回-1
	 */
	int sgn_Positive(double d);

	/**
	 * @brief 五次多项式求解
	 * @param t 运动队列中每个点的时间点
	 * @param coe get_Quintic_Coe()计算得到的五次多项式系数
	 * @param qptr 插补得到的运动队列指针
	 */
	void Quintic_Polynomi(double r, Quintic_Coe &coe, double *qptr);
	/**
	 * @brief 计算直线插补系数
	 * @param q0 初始位置
	 * @param q1 目标位置
	 * @param Ts 插补周期
	 * @param maxVel 最大速度
	 * @param maxAcc 最大加速度
	 * @param maxDecel 最大减速度
	 * @param maxJerk 最大雅可比速度
	 * @param seq 插补得到的运动队列
	 * @param beginVel 初始速度
	 * @param beginAcc 初始加速度
	 */
	void calc_Interp_5_1_5(double q0, double q1, double Ts,
						   double maxVel, double maxAcc, double maxDecel, double maxJerk, std::deque<double> &seq, double beginVel, double beginAcc);
};

#endif /* GENERAL_6S_H_ */
