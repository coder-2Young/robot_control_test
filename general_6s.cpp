#include "general_6s.h"
#include <iostream>

// 机械臂各个关节初始偏置
const double joint_bias[] = {
	13.1463,
	-17.7446,
	-81.168,
	0,
	16.3205 + 90,
	140.364,
};

//
double direction[] = {1, 1, -1, -1, 1, 1};

// 机械臂物理方向，逆时针为正，顺时针为负
const int joint_direction[] = {1, -1, 1, -1, 1, -1};

General_6S *g_general_6s = nullptr;

VectorXd General_6S::tr_2_MCS(MatrixXd m) //转移矩阵转直角坐标
{
	VectorXd posMCS(6);
	posMCS << m.block(0, 3, 3, 1), tr_2_rpy(m);
	return posMCS;
}

void General_6S::calc_forward_kin(const VectorXd &posACS, MatrixXd &transMatrix)
{
	VectorXd acs_rad(6);

	transMatrix = (MatrixXd::Identity(4, 4));

	for (int i = 0; i < 6; i++)
	{
		acs_rad[i] = deg2rad(posACS[i]);
		MatrixXd X(4, 4); /**< @brief Gets or sets the matrix regarding X axis transformations. */
		MatrixXd Z(4, 4); /**< @brief Gets or sets the matrix regarding Z axis transformations. */
		Z << Eigen::AngleAxisd(dh.theta[i] + acs_rad[i], Vector3d::UnitZ()).toRotationMatrix(), (MatrixXd(3, 1) << 0.0, 0.0, dh.d[i]).finished(),
			MatrixXd::Zero(1, 3), 1;
		X << Eigen::AngleAxisd(dh.alpha[i], Vector3d::UnitX()).toRotationMatrix(), (MatrixXd(3, 1) << dh.a[i], 0.0, 0.0).finished(),
			MatrixXd::Zero(1, 3), 1;
		transMatrix = transMatrix * Z * X;
	}
}

void General_6S::calc_inverse_kin(const MatrixXd &transMatrix, const VectorXd &posLast, VectorXd &posACS)
{
	double d1, d2, d4, d6, a1, a2, a3;
	if ((transMatrix.rows() != 4) || (transMatrix.cols() != 4) || (posLast.rows() != 6) || (posACS.rows() != 6))
	{
		return;
	}
	//vars defined temporarily
	d1 = dh.d[0];
	d2 = dh.d[1];
	d4 = dh.d[3];
	d6 = dh.d[5];
	a1 = dh.a[0];
	a2 = dh.a[1];
	a3 = dh.a[2];
	MatrixXd T(transMatrix);

	/*		2021-11-26新增		*/
	VectorXd pLast(6); //读到的角度，经下面处理后，成为theta值
	for (int i = 0; i < 6; i++)
		pLast(i) = deg2rad(posLast(i));
	/*		2021-11-26新增结束		*/

	double nx(T(0, 0)), ny(T(1, 0)), nz(T(2, 0));
	double ox(T(0, 1)), oy(T(1, 1)), oz(T(2, 1));
	double ax(T(0, 2)), ay(T(1, 2)), az(T(2, 2));
	double px(T(0, 3)), py(T(1, 3)), pz(T(2, 3));

	//读取theat5偏移量
	double theta5_offset = dh.theta[4];

	//solve for theta1
	double theta1_1, theta1_2;
	double n = px - d6 * ax;
	double m = py - d6 * ay;
	double temp_var = pow(m, 2) + pow(n, 2) - pow(d2, 2);
	if (temp_var < 0.0L)
	{
		return;
	}
	if ((fabs((py - d6 * ay)) < 10e-13) && (fabs((px - d6 * ax)) < 10e-13))
	{
		theta1_1 = pLast(0);
		theta1_2 = theta1_1;
	}
	else
	{
		theta1_1 = atan2(n * d2 + m * sqrt(temp_var), -m * d2 + n * sqrt(temp_var));
		theta1_2 = atan2(n * d2 - m * sqrt(temp_var), -m * d2 - n * sqrt(temp_var));
	}
	double theta1;

	/*if (theta1_1 <= 0.0L)
		theta1_2 = theta1_1 + M_PI;
	else
		theta1_2 = theta1_1 - M_PI;*/

	if (fabs(theta1_1 - pLast(0)) > fabs(theta1_2 - pLast(0)))
		theta1 = theta1_2;
	else
		theta1 = theta1_1;

	// the limit of theta1 according to the reference
	posACS(0) = theta1;
	//solve for theta3

	double k1 = cos(theta1) * px + sin(theta1) * py - a1 - d6 * (cos(theta1) * ax + sin(theta1) * ay);
	double k2 = pz - d1 - d6 * az;
	double k3 = pow(k1, 2) + pow(k2, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2);
	double k4 = k3 / (2 * a2);
	temp_var = pow(a3, 2) + pow(d4, 2) - pow(k4, 2);
	if (temp_var < 0.0L)
	{
		return;
	}
	double delta = sqrt(pow(a3, 2) + pow(d4, 2) - pow(k4, 2));
	double theta3_1 = atan2(d4, a3) + atan2(delta, k4);
	double theta3_2 = atan2(d4, a3) - atan2(delta, k4);
	double theta3;

	if (fabs(theta3_1 - pLast(2)) > fabs(theta3_2 - pLast(2)))
		theta3 = theta3_2;
	else
		theta3 = theta3_1;

	// the limit of theta3 according to the reference
	posACS(2) = theta3;

	//solve for theta2
	k1 = cos(theta1) * px + sin(theta1) * py - a1 - d6 * (cos(theta1) * ax + sin(theta1) * ay);
	k2 = -d1 + pz - d6 * az;
	double a = d4 * cos(theta3) - a3 * sin(theta3);
	double b = d4 * sin(theta3) + a2 + a3 * cos(theta3);
	//已经加入了theta2的offset -pi/2    theta(运算) = theta(电机) - pi/2
	double theta2_1;
	if ((fabs(a * k1 + b * k2) < 10e-13) && (fabs(b * k1 - a * k2) < 10e-13))
		theta2_1 = pLast(1);
	else
		theta2_1 = atan2((a * k1 + b * k2), (b * k1 - a * k2)) - M_PI / 2.0;
	double theta2;

	theta2 = theta2_1;
	// the limit of theta2 according to the reference
	posACS(1) = theta2;

	//solve for theta4

	k1 = sin(theta1) * ax - cos(theta1) * ay;
	k2 = cos(theta1) * cos(theta2 + M_PI / 2.0 + theta3) * ax + sin(theta1) * cos(theta2 + M_PI / 2.0 + theta3) * ay + sin(theta2 + M_PI / 2.0 + theta3) * az;

	double theta4;
	double theta4_2;
	//此处的判断阈值不能过小，过小的话，当0/0时，它无法识别出来

	if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	{
		theta4 = pLast(3);
		//cout << "A" << endl;
	}
	else
	{
		double theta4_1 = atan2(k1, k2);
		if (theta4_1 > 0.0L)
			theta4_2 = theta4_1 - M_PI;
		else
			theta4_2 = theta4_1 + M_PI;
		if (fabs(theta4_1 - pLast(3)) > fabs(theta4_2 - pLast(3)))
			theta4 = theta4_2;
		else
			theta4 = theta4_1;
	}

	// the limit of theta4 according to the reference
	posACS(3) = theta4;

	//solve for theta5
	double k1_1 = sin(theta1) * sin(theta4) + cos(theta1) * cos(theta4) * cos(theta2 + M_PI / 2.0 + theta3);
	double k1_2 = -cos(theta1) * sin(theta4) + sin(theta1) * cos(theta4) * cos(theta2 + M_PI / 2.0 + theta3);
	double k1_3 = cos(theta4) * sin(theta2 + M_PI / 2.0 + theta3);
	k1 = k1_1 * ax + k1_2 * ay + k1_3 * az;
	k2 = cos(theta1) * sin(theta2 + M_PI / 2.0 + theta3) * ax + sin(theta1) * sin(theta2 + M_PI / 2.0 + theta3) * ay - cos(theta2 + M_PI / 2.0 + theta3) * az;
	double theta5_1;
	if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	{
		theta5_1 = pLast(4);
	}
	else
	{
		if (fabs(theta5_offset - M_PI / 2) < 10e-4) //判断第五轴是垂直还是平行
		{
			theta5_1 = atan2(-k1, k2) - M_PI / 2.0;
		}
		else if (fabs(theta5_offset) < 10e-4)
		{
			theta5_1 = atan2(-k1, k2);
		}
	}
	double theta5;

	theta5 = theta5_1;
	// the limit of theta5 according to the reference
	posACS(4) = theta5;

	//solve for theta6
	double k1_4 = sin(theta4) * cos(theta2 + M_PI / 2.0 + theta3) * cos(theta1) - cos(theta4) * sin(theta1);
	double k1_5 = sin(theta4) * cos(theta2 + M_PI / 2.0 + theta3) * sin(theta1) + cos(theta4) * cos(theta1);
	double k1_6 = sin(theta4) * sin(theta2 + M_PI / 2.0 + theta3);
	k2 = -k1_4 * ox - k1_5 * oy - k1_6 * oz;
	k1 = -k1_4 * nx - k1_5 * ny - k1_6 * nz;

	double theta6_1;
	if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
		theta6_1 = pLast(5);
	else
		theta6_1 = atan2(k1, k2);
	double theta6;

	theta6 = theta6_1;
	// the limit of theta6 according to the reference
	posACS(5) = theta6;

	for (int i = 0; i < 6; i++)
		posACS[i] = rad2deg(posACS[i]);
}

void General_6S::move_joint_interp(const VectorXd &targetPoint,
								   const VectorXd &originPoint, const VectorXd &velCurrent, const VectorXd &accCurrent, double Ts, double velPerc,
								   double accPerc, double decPerc, double jerkPerc, std::deque<double> &deque) //输入的是角度值
{
	int n = 6;
	VectorXd posacs(n);
	int mcsDimension = 6;
	VectorXd posmcs(mcsDimension); //正解求直角坐标位置

	//	若有位置命令未执行完毕，等待各轴进入空闲状态
	//	MC_WaitIsFree();

	VectorXd posOrigin(originPoint);
	VectorXd posTarget(targetPoint);

	//计算各轴角位移偏移量
	VectorXd posOffset(n);
	posOffset = posTarget - posOrigin;

	VectorXd t(n);	 //若全程匀速，各轴时间
	double T = 0;	 //关节运动最长的那个时间
	VectorXd tA(n);	 //各轴加速时间
	double tAcc = 0; //关节加速过程最长的那个时间
	VectorXd tD(n);	 //各轴加速时间
	double tDec = 0; //关节加速过程最长的那个时间

_AdjustVel:

	velPerc = velPerc / 100;
	accPerc = accPerc / 100;
	decPerc = decPerc / 100;

	//第一步：确定加（减）速段时间tAcc
	for (int ai = 0; ai != n; ++ai) /*1-n轴*/
	{
		tA[ai] = fabs((velPerc * 1.0 * (motor_param.RatedVel[ai]) * sgn_Positive(posOffset[ai]) - velCurrent[ai]) / (accPerc * (motor_param.maxAcc[ai]) * (motor_param.RatedVel[ai])));
		tD[ai] = (velPerc * 1.0) / (decPerc * fabs(motor_param.maxDecel[ai]));

		if (tA[ai] > (velPerc * 1.0) / (accPerc * fabs(motor_param.maxAcc[ai])))
		{
			tA[ai] = (velPerc * 1.0) / (accPerc * fabs(motor_param.maxAcc[ai]));
		}
	}

	tAcc = tA[0];
	tDec = tD[0];
	for (int ai = 1; ai != n; ++ai) //2-n轴
	{
		if ((tAcc > tA[ai] || tAcc < 2 * Ts) && (fabs(posOffset[ai]) > 0.1) && tA[ai] > Ts)
		{
			tAcc = tA[ai];
		}
		if (tDec > tD[ai] && (fabs(posOffset[ai]) > 0.1))
		{
			tDec = tD[ai];
		}
	}

	for (int ai = 0; ai != n; ++ai) //1-n轴
	{
		if ((tAcc < tA[ai]) && (fabs(posOffset[ai]) > 0.1))
		{
			tAcc = tA[ai];
		}

		if ((tDec < tD[ai]) && (fabs(posOffset[ai]) > 0.1))
		{
			tDec = tD[ai];
		}
	}
	if (!tAcc || !tDec)
	{
		return;
	}

	//第二步：若全程视作匀速运动，确定各轴最大运动时间T

	for (int ai = 0; ai < n; ++ai) /*1-n轴*/
	{
		if (velPerc > 0.998 && (motor_param.maxRotSpeed[ai] < 1.05))
		{
			t[ai] = fabs(posOffset[ai] - tA[ai] / 2 * velCurrent[ai]) / (velPerc * (motor_param.RatedVel[ai]) * 0.998);
		}
		else
		{
			t[ai] = fabs(posOffset[ai] - tA[ai] / 2 * velCurrent[ai]) / (velPerc * (motor_param.RatedVel[ai]));
		}
	}
	//比较时间大小，取最大值赋值给T
	T = t[0];
	for (int ai = 1; ai != n; ++ai) //2-n轴
	{
		if (T < t[ai])
		{
			T = t[ai];
		}
	}
	if (!T)
	{
		return;
	}

	//第三步：确定匀速段时间tConst
	double tf = T + tAcc / 2 + tDec / 2; //最终整个过程运行时间为tf
	double tConst = tf - tAcc - tDec;
	if (tConst < 0)
	{
		//todo: 不采用速度减半方法，而是改为只有变速段，没有匀速段处理
		//速度减半
		velPerc = velPerc * 98;
		accPerc = accPerc * 98;
		decPerc = decPerc * 98;
		goto _AdjustVel;
		//return PLCOPEN_MOVE_INSTRUCT_NOT_APPLICABLE;
	}

	//第四步：确定加速段、匀速段、减速段的插补步数（N1,N2,N3），并确定最终的运行时间
	size_t N1 = static_cast<size_t>(tAcc / Ts);
	size_t N2 = static_cast<size_t>(tConst / Ts);
	size_t N3 = static_cast<size_t>(tDec / Ts);
	if (N1 == 0)
	{
		N1 = 1;
	}
	tAcc = N1 * Ts;
	tConst = N2 * Ts;
	tDec = N3 * Ts;
	T = tConst + tAcc / 2 + tDec / 2;

	//第五步：计算各轴最终运行时的平均速度(也即匀速段的速度velConst，注意这个速度与设定的速度可能不一样)
	VectorXd velConst(n);			//有正负号
	for (int ai = 0; ai != n; ++ai) /*1-n轴*/
	{
		velConst[ai] = (posOffset[ai] - tAcc / 2 * velCurrent[ai]) / T;
	}

	//第六步：轨迹插补，其中：用五次多项式对加速段和减速段进行插补
	VectorXd posInterp(n); //用于暂存各轴某周期的插补点位置
	VectorXd pLast(n);

	pLast = deg2rad(originPoint);

	//(1)加速段插补 [ 第 0 ~ N1 点]
	//计算五次多项式系数
	VectorXd accPosEnd(n);				   //各轴加速段的结束点位置
	std::deque<Quintic_Coe> coeQuintic(n); //各个轴的五次多项式

	for (int ai = 0; ai != n; ++ai) /*1-n轴*/
	{
		accPosEnd[ai] = posOrigin[ai] + (velConst[ai] + velCurrent[ai]) * tAcc / 2;
		get_Quintic_Coe(posOrigin[ai], accPosEnd[ai], velCurrent[ai], velConst[ai], accCurrent[ai], 0,
						tAcc, coeQuintic[ai]);
	}
	//生成链表
	for (size_t i = 0; i != (N1 + 1); ++i) //i用于记录加速段插补点的序号
	{
		double t = i * Ts;				//时间t
		for (int ai = 0; ai != n; ++ai) /*1-n轴*/
		{
			Quintic_Polynomi(t, coeQuintic[ai], &(posInterp[ai]));
			posacs[ai] = deg2rad(posInterp[ai]);
		}

		for (int ai = 0; ai != n; ++ai) /*1-n轴*/
		{
			deque.push_back(posInterp[ai]);
		}
		pLast = deg2rad(posInterp);
	}

	//(2)匀速段插补 [ 第 (N1+1) ~ (N1+N2) 点]
	//生成链表
	for (size_t j = 1; j != (N2 + 1); ++j) //j用于记录匀速段插补点的序号
	{
		for (int ai = 0; ai != n; ++ai) /*1-n轴*/
		{
			posInterp[ai] += velConst[ai] * Ts;
			posacs[ai] = deg2rad(posInterp[ai]);
		}

		for (int ai = 0; ai != n; ++ai) /*1-n轴*/
		{
			deque.push_back(posInterp[ai]);
		}
		pLast = deg2rad(posInterp);
	}

	//(3)减速段插补 [ 第 (N1+N2+1) ~ (N1+N2+N3) 点]
	//计算五次多项式系数
	for (int ai = 0; ai != n; ++ai) /*1-n轴*/
	{
		get_Quintic_Coe(posInterp[ai], posTarget[ai], velConst[ai], 0, 0, 0,
						tDec, coeQuintic[ai]);
	}
	//生成链表
	for (size_t k = 1; k != (N3 + 1); ++k) //k用于记录减速段插补点的序号
	{
		double t = k * Ts;				//时间t
		for (int ai = 0; ai != n; ++ai) /*1-n轴*/
		{
			Quintic_Polynomi(t, coeQuintic[ai], &(posInterp[ai]));
			posacs[ai] = deg2rad(posInterp[ai]);
		}

		for (int ai = 0; ai != n; ++ai) /*1-n轴*/
		{
			deque.push_back(posInterp[ai]);
		}
		pLast = deg2rad(posInterp);
	}
	return;
}
void General_6S::move_line_interp(const VectorXd &targetPoint,
								  const VectorXd &originPoint, const VectorXd &originACS, double velCurrent, double accCurrent,
								  double Ts, double maxVelper, double maxAccper, double maxDecelper,
								  double maxJerk, std::deque<double> &deque)
{
	int n = 6;
	int mcsDimension = 6;

	int axisNum;
	VectorXd targetACS(n);

	bool moveDirection = true;

	//	若有位置命令未执行完毕，等待各轴进入空闲状态
	//	MC_WaitIsFree();

	//计算路径长度
	double displacement = sqrt(pow((targetPoint[0] - originPoint[0]), 2) + pow((targetPoint[1] - originPoint[1]), 2) + pow((targetPoint[2] - originPoint[2]), 2));
	std::deque<double> rlst;
	std::deque<double> rlstMcs;
	std::deque<double> rlstAcs;
	double maxVelAcs, maxAccAcs, maxDecelAcs, displacementAcs;

	if (displacement > 0.2)
	{
		if (sqrt(maxAccper * displacement) < (0.98 * maxVelper))
		{
			for (int i = 0; i < 1000; i++)
			{
				maxVelper = 0.98 * maxVelper;
				maxAccper = 0.98 * maxAccper;
				maxDecelper = 0.98 * maxDecelper;
				if (sqrt(maxAccper * displacement) >= maxVelper)
				{
					std::cout << "sqrt(maxAccper * displacement) >= maxVelper" << std::endl;
					break;
				}
			}
		}
		calc_Interp_5_1_5(0, 1, Ts, maxVelper / displacement, maxAccper / displacement,
						  maxDecelper / displacement, maxJerk / displacement, rlstMcs, velCurrent / displacement, accCurrent / displacement);
		calc_inverse_kin(rpy_2_tr(targetPoint), originACS, targetACS);
		// 中间运算是偏离值的计算，不需要转化为实际值
		// for (int i = 0; i < 6; i++)
		// {
		// 	targetACS(i) = joint_bias[i] + targetACS(i) * direction[i]; // 得到各关节角度值，为机械臂实际各关节角度
		// }
		displacementAcs = fabs(targetACS[0] - originACS[0]);
		axisNum = 0;
		for (int ai = 1; ai != n; ++ai) //2-n轴
		{
			if (displacementAcs < fabs(targetACS[ai] - originACS[ai]))
			{
				displacementAcs = fabs(targetACS[ai] - originACS[ai]);
				axisNum = ai;
			}
		}

		if (displacementAcs < 0.0001)
		{
			std::cout << "displacementAcs < 0.0001" << std::endl;
			return;
		}
		else
		{
			maxVelAcs = motor_param.RatedVel[axisNum];
			maxAccAcs = motor_param.maxAcc[axisNum] * motor_param.RatedVel[axisNum];
			maxDecelAcs = motor_param.maxDecel[axisNum] * motor_param.DeRatedVel[axisNum];

			if (sqrt(maxAccAcs * displacementAcs) < maxVelAcs)
			{
				maxVelAcs = sqrt(maxAccAcs * displacementAcs);
			}
		}
		calc_Interp_5_1_5(0, 1, Ts, maxVelAcs / displacementAcs, maxAccAcs / displacementAcs,
						  maxDecelAcs / displacementAcs, maxJerk / displacementAcs, rlstAcs, velCurrent / displacement, accCurrent / displacement);
		rlst = rlstMcs;
	}
	else
	{
		calc_inverse_kin(rpy_2_tr(targetPoint), originACS, targetACS);
		// 中间运算是偏离值的计算，不需要转化为实际值
		// for (int i = 0; i < 6; i++)
		// {
		// 	targetACS(i) = joint_bias[i] + targetACS(i) * direction[i]; // 得到各关节角度值，为机械臂实际各关节角度
		// }
		displacement = fabs(targetACS[0] - originACS[0]);
		axisNum = 0;
		for (int ai = 1; ai != n; ++ai) //2-n轴
		{
			if (displacement < fabs(targetACS[ai] - originACS[ai]))
			{
				displacement = fabs(targetACS[ai] - originACS[ai]);
				axisNum = ai;
			}
		}
		if (displacement < 0.0001)
		{
			std::cout << "else displacementAcs < 0.0001" << std::endl;
			return;
		}
		else
		{
			maxVelper = maxVelper / decare.maxvel * (motor_param.RatedVel[axisNum]);
			maxAccper = maxAccper / (decare.maxacc * decare.maxvel) * (motor_param.maxAcc[axisNum] * motor_param.RatedVel[axisNum]);
			maxDecelper = maxDecelper / (decare.maxdec * decare.maxvel) * (motor_param.maxDecel[axisNum] * motor_param.DeRatedVel[axisNum]);
			if (sqrt(maxAccper * displacement) < maxVelper)
			{
				maxVelper = sqrt(maxAccper * displacement);
			}
			if (maxAccper / maxVelper > 2)
			{
				maxAccper = 2 * maxVelper;
				maxDecelper = 2 * maxVelper;
			}
		}
		calc_Interp_5_1_5(0, 1, Ts, maxVelper / displacement, maxAccper / displacement,
						  maxDecelper / displacement, maxJerk / displacement, rlst, velCurrent / displacement, accCurrent / displacement);
	}

	//球面线性插补
	Matrix3d mOrigin = rpy_2_r(originPoint.segment(3, 3));
	Matrix3d mTarget = rpy_2_r(targetPoint.segment(3, 3));

	Vector3d pOrigin = originPoint.head(3);
	Vector3d pTarget = targetPoint.head(3);
	Vector3d pr;
	MatrixXd Tr(4, 4);
	VectorXd posr(mcsDimension);
	VectorXd posACS(n);
	VectorXd posAcACS(n);
	VectorXd pLast(n);
	Quaternion<double> qr;
	Quaternion<double> qTarget(mTarget);
	Quaternion<double> qOrigin(mOrigin);

	for (int i = 0; i < 11; i++)
	{
		double r = i / 10.0;
		qr = qOrigin.slerp(r, qTarget);
		pr = pOrigin * (1 - r) + r * pTarget;
		Tr << qr.toRotationMatrix(), pr, MatrixXd::Zero(1, 3), 1;
		posr.head(6) << tr_2_MCS(Tr);
		if (i == 0)
		{
			pLast = originACS;
		}
		calc_inverse_kin(rpy_2_tr(posr), pLast, posACS);
		// 中间运算是偏离值的计算，不需要转化为实际值
		// for (int i = 0; i < 6; i++)
		// {
		// 	posACS(i) = joint_bias[i] + posACS(i) * direction[i]; // 得到各关节角度值，为机械臂实际各关节角度
		// }
		pLast = posACS;
	}
	for (std::deque<double>::iterator rit = rlst.begin(); rit != rlst.end(); ++rit)
	{
		double r = *rit;
		if (moveDirection)
		{
			qr = qOrigin.slerp(r, qTarget);
		}
		else
		{
			qr = qOrigin.slerpLongWay(r, qTarget);
		}
		pr = pOrigin * (1 - r) + r * pTarget;
		Tr << qr.toRotationMatrix(), pr, MatrixXd::Zero(1, 3), 1;

		posr.head(6) << tr_2_MCS(Tr);

		if (rit == rlst.begin())
		{
			pLast = originACS;
		}
		calc_inverse_kin(rpy_2_tr(posr), pLast, posACS);
		for (int i = 0; i < 6; i++)
		{
			posAcACS(i) = joint_bias[i] + posACS(i) * direction[i]; // 得到各关节角度值，为机械臂实际各关节角度
			deque.push_back(posAcACS[i]);
		}

		// for (int ai = 0; ai != n; ++ai) //1-n轴
		// {
		// 	deque.push_back(posAcACS[ai]);
		// }
		pLast = posACS;
	}
	std::cout << "end"<< std::endl;
	return;
}

MatrixXd General_6S::rpy_2_tr(VectorXd posMCS) //直角坐标转转移矩阵
{
	MatrixXd tr(4, 4);
	tr << rpy_2_r(posMCS.segment(3, 3)), posMCS.head(3), MatrixXd::Zero(1, 3), 1;
	return tr;
}
Vector3d General_6S::tr_2_rpy(MatrixXd m) //转移矩阵转位姿
{
	Vector3d rpy;
	rpy(0) = atan2(-m(1, 2), m(2, 2));
	double sr = sin(rpy(0, 0));
	double cr = cos(rpy(0, 0));
	rpy(1) = atan2(m(0, 2), cr * m(2, 2) - sr * m(1, 2));
	rpy(2) = atan2(-m(0, 1), m(0, 0));
	return rpy;
}
MatrixXd General_6S::rpy_2_r(Vector3d rpy) //位姿转旋转矩阵
{
	return Eigen::AngleAxisd(rpy(0), Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3d::UnitZ()).toRotationMatrix();
}

void General_6S::get_Quintic_Coe(double q0, double qf, double dq0, double dqf,
								 double ddq0, double ddqf, double tf, Quintic_Coe &Coe)
{
	Coe.a0 = q0;
	Coe.a1 = dq0;
	Coe.a2 = ddq0 / 2.0;
	Coe.a3 = (20 * qf - 20 * q0 - (8 * dqf + 12 * dq0) * tf - (3 * ddq0 - ddqf) * tf * tf) / (2 * pow(tf, 3));
	Coe.a4 = (30 * q0 - 30 * qf + (14 * dqf + 16 * dq0) * tf + (3 * ddq0 - 2 * ddqf) * tf * tf) / (2 * pow(tf,
																										   4));
	Coe.a5 = (12 * qf - 12 * q0 - (6 * dq0 + 6 * dqf) * tf - (ddq0 - ddqf) * tf * tf) / (2 * pow(tf, 5));
}

int General_6S::sgn_Positive(double d)
{
	return d < -10e-5 ? -1 : 1;
}
//五次多项式求解
void General_6S::Quintic_Polynomi(double r, Quintic_Coe &coe, double *qptr)
{
	if (qptr)
		*qptr = coe.a0 + coe.a1 * r + coe.a2 * pow(r, 2) + coe.a3 * pow(r, 3) + coe.a4 * pow(r, 4) + coe.a5 * pow(r, 5);
}
void General_6S::calc_Interp_5_1_5(double q0, double q1, double Ts,
								   double maxVel, double maxAcc, double maxDecel, double maxJerk,
								   std::deque<double> &seq, double beginVel, double beginAcc)
{
	double dis = q1 - q0;
	if (!maxVel)
		return;

	//第一步：确定加（减）速段时间tAcc
	double tAcc = fabs((maxVel - beginVel) / maxAcc);
	double tDecel = fabs(maxVel / maxDecel);

	if (tAcc > maxVel / maxAcc)
	{
		tAcc = maxVel / maxAcc;
	}

	//第二步：若全程视作匀速运动，确定最大运动时间T
	double T = fabs((dis - tAcc / 2 * beginVel) / maxVel);
	if (!T)
	{
		return;
	}

	//第三步：确定匀速段时间tConst
	double tf = tAcc / 2 + T + tDecel / 2; //最终整个过程运行时间为tf
	double tConst = tf - tAcc - tDecel;

	//判断是否有匀速段
	if (tConst < 0.75 * Ts)
		tConst = 0;
	{
		//第四步：确定加速段、匀速段、减速段的插补步数（N1,N2,N3）,并确定最终运动时间
		size_t N1 = static_cast<size_t>(tAcc / Ts);
		size_t N2 = static_cast<size_t>(tConst / Ts);
		size_t N3 = static_cast<size_t>(tDecel / Ts);
		if (N1 == 0)
		{
			N1 = 1;
		}
		tAcc = N1 * Ts;
		tDecel = N3 * Ts;
		tConst = N2 * Ts;

		T = tConst + tAcc / 2 + tDecel / 2;

		//第五步：计算轴最终运行时的平均速度，就是有符号的maxVel
		double velConst = (dis - tAcc / 2 * beginVel) / T; //有正负号

		//第六步：轨迹插补，其中：用五次多项式对加速段和减速段进行插补
		seq.clear();
		double posInterp = 0; //用于暂存轴某周期的插补点位置

		//(1)加速段插补 [ 第 0 ~ N1 点]
		//计算五次多项式系数
		double accPosEnd; //轴加速段的结束点位置
		Quintic_Coe coe;  //轴的五次多项式
		accPosEnd = q0 + (velConst + beginVel) * tAcc / 2;
		get_Quintic_Coe(q0, accPosEnd, beginVel, velConst, beginAcc, 0, tAcc, coe);

		//生成链表
		for (size_t i = 0; i != (N1 + 1); ++i) //i用于记录加速段插补点的序号
		{
			Quintic_Polynomi(i * Ts, coe, &posInterp);
			seq.push_back(posInterp);
		}

		//(2)匀速段插补 [ 第 (N1+1) ~ (N1+N2) 点]
		//生成链表
		for (size_t j = 1; j != (N2 + 1); ++j) //j用于记录匀速段插补点的序号
		{
			posInterp += velConst * Ts;
			seq.push_back(posInterp);
		}

		//(3)减速段插补 [ 第 (N1+N2+1) ~ (N1+N2+N3) 点]
		//计算五次多项式系数
		get_Quintic_Coe(posInterp, q1, velConst, 0, 0, 0, tDecel, coe);

		//生成链表
		for (size_t k = 1; k != (N3 + 1); ++k) //k用于记录减速段插补点的序号
		{
			Quintic_Polynomi(k * Ts, coe, &posInterp);
			seq.push_back(posInterp);
		}
	}
	return;
}
