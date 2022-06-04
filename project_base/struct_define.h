#ifndef PROJECT_BASE_STRUCT_DEFINE_H_
#define PROJECT_BASE_STRUCT_DEFINE_H_

#include <vector>
#include <map>
/**
 * @brief DH参数结构体
 */
struct DH_param  
{
	double alpha[6];
	double d[6];
	double a[6];
	double theta[6];
	DH_param()
	{
		for(int i=0;i<6;i++)
		{
			alpha[i] = 0;
			d[i] = 0;
			a[i] = 0;
			theta[i] = 0;
		}
	}
};
/**
 * @brief 笛卡尔参数结构体
 */
struct Decare_Para
{
	double maxvel;
	double maxacc;
	double maxdec;
	double maxjerk;
	Decare_Para()
	{
		maxvel = 0;
		maxacc = 0;
		maxdec = 0;
		maxjerk = 0;
	}
};
/**
 * @brief 编码器参数结构体
 */
struct Encoder_Param
{
	int direction[6];///运动方向
	double reducRatio[6];///减速比
	int encoderResolution[6];///编码器分辨率
	double singleTurnEncoder[6];///编码器记录值
	double deviation[6];///相对主轴位置偏差
	Encoder_Param()
	{
		for(int i=0;i<6;i++)
		{
			direction[i] = 0;
			reducRatio[i] = 0;
			encoderResolution[i] = 0;
			singleTurnEncoder[i] = 0;
			deviation[i] = 0;
		}
	}
};
/**
 * @brief 电机参数结构体
 */
struct Motor_Param
{
	Encoder_Param encoder;
	double RatedVel_rpm[6];///额定转速，单位转每分
	double RatedVel[6];///额定正转速
	double DeRatedVel[6];///额定反转速
	double maxAcc[6];///最大加速度,额定转速的倍数
	double maxDecel[6];///最大减速度,额定转速的倍数
	double maxRotSpeed[6];///最大转速,额定转速的倍数
	Motor_Param()
	{
		Encoder_Param();
		for(int i=0;i<6;i++)
		{
			RatedVel_rpm[i] = 0;
			RatedVel[i] = 0;
			DeRatedVel[i] = 0;
			maxAcc[i] = 0;
			maxDecel[i] = 0;
			maxRotSpeed[i] = 0;
		}
	}
};
/**
 * @brief 二次多项式的系数
 */
typedef struct tag_Trapezoidal_Coe
{
	double a0; 
	double a1; 
	double a2; 
} trapezoidal_Coe; 
/**
 * @brief 五次多项式的系数
 */
typedef struct tag_Quintic_Coe
{
    double a0; 
    double a1; 
    double a2; 
    double a3; 
    double a4; 
    double a5; 
} Quintic_Coe; 

/*		2021-11-26新增		*/
/**
 * @brief pdo数据类型的枚举
 * char_ : signed char
 * short_ : signed short
 * int_ : signed int
 * uchar_ : unsigned char
 * ushort_ : unsigned short
 * uint_ : unsigned int
 */
enum pdo_object_type{char_,short_,int_,uchar_,ushort_,uint_};

/**
 * @brief 用于读写pdo的数据类型
 */
union Pdo_value
{
	Pdo_value()
	{
		char_value = 0;
		short_value = 0;
		int_value = 0;
		uchar_value = 0;
		ushort_value = 0;
		uint_value = 0;
	}
	signed char	    char_value;
	signed short    short_value;
	signed int      int_value;
	unsigned char	    uchar_value;
	unsigned short    ushort_value;
	unsigned int      uint_value;
};
/*		2021-11-26新增结束		*/
#endif /* PROJECT_BASE_STRUCT_DEFINE_H_ */
