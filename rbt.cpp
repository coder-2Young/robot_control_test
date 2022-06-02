/*
 * test.cpp
 *
 *  Created on: Nov 24, 2020
 *      Author: liuchongyang
 */
#include <unistd.h>
#include <cmath>
#include "general_6s.h"
#include <iostream>
#include <map>
#include <time.h>
#include "server.cpp"
#include <unistd.h>
#include <thread>
#include <chrono>   
using namespace std;
using namespace chrono;


extern General_6S *g_general_6s;

extern void start_ecm(int argc, char *argv[]);
extern void loop_display();

#define PI 3.1415926

typedef void (*FnPtr)(void);
typedef string (*StrFnPtr)(void);
typedef string (*StrFnStrPtr)(string);

// 机械臂是否在动作
bool robot_is_moving = false;

// 机械臂各个关节额定力矩
const double joints_rated_torque[] = {1270, 1270, 640, 318, 159, 159};

// 机械臂各个关节减速比
const double joints_reduce_ratio[] = {81, 101, 63.462, 68.966, 81.25, 40.625};

// 机械臂各个关节初始偏置
const double joint_bias[] = {
    13.1463,
    -17.7446,
    -81.168,
    0,
    16.3205 + 90,
    140.364,
};

// 编码器分辨率
const double encoder_resolution = 131072;

// 机械臂物理方向，逆时针为正，顺时针为负
const int joint_direction[] = {1, -1, 1, -1, 1, -1};

const int refer_direction[] = {1, -1, -1, 1, 1, 1};

// const int
// 调整关节加和
double joint_add(int axis, double initial, double deg)
{
    return initial + deg * joint_direction[axis];
}

// 获取机械臂真实角度
double joint_deg(int axis, double delta_deg)
{
    return joint_bias[axis] + delta_deg * joint_direction[axis];
}

// 获取机械臂变角
double joint_delta(int axis, double real_joint_deg)
{
    return (real_joint_deg - joint_bias[axis]) * joint_direction[axis];
}

void joint_move()
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    char joint_info[255];
    cout << "逆时针旋转角度为正，顺时针旋转角度为负，输入角度格式为：轴;角度" << endl;
    cin >> joint_info;
    char joint_axis_str[2];
    char joint_deg_str[255];
    // 读取关节轴
    joint_axis_str[0] = joint_info[0];
    joint_axis_str[1] = '\0';
    // 读取关节角度
    for (int i = 0; i < 255; i++)
    {
        cout << joint_info << endl;
        if ('\0' == joint_info[i + 2])
        {
            joint_deg_str[i] = '\0';

            break;
        }
        joint_deg_str[i] = joint_info[i + 2];
    }
    int axis = atoi(joint_axis_str);
    double deg = atof(joint_deg_str);
    cout << "逆时针旋转角度为正，顺时针旋转角度为负，输入角度格式为：轴;角度" << endl;
    cout << "轴:" << axis << endl;
    cout << "转角:" << deg << endl;
    cout << "是否确定？yes|no" << endl;
    cin >> joint_info;
    if (0 != strcmp(joint_info, "yes"))
    {
        cout << "取消转动操作，程序退出";
        robot_is_moving = false;
        return;
    }

    VectorXd target_point_joint(6);      //目标位置,角度制
    VectorXd origin_point_joint_test(6); //初始位置,角度制
    VectorXd vel_current_joint_test(6);  //当前速度,角度制
    VectorXd acc_current_joint_test(6);  //当前加速度,角度制
    double pos_cur_ang[6];               //当前位置角度值,角度制

    for (int i = 0; i < 6; i++)
    {
        pos_cur_ang[i] = g_general_6s->get_actual_position(i); //获取当前位置角度值
        origin_point_joint_test(i) = pos_cur_ang[i];           //当前位置作为起始位置
        cout << "joint" << i << "--" << pos_cur_ang[i] << endl;
    }

    // 机器人标准初始姿态位置(0位置)
    for (int i = 0; i < 6; i++)
    {
        if (axis == i)
        {
            target_point_joint(i) = joint_add(i, pos_cur_ang[i], deg);
        }
        else
        {
            target_point_joint(i) = pos_cur_ang[i];
        }
    }

    vel_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前速度
    acc_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前加速度
    double Ts_joint_test = 0.001;               //设置运动周期
    double velPerc_joint_test = 2;              //设置速度百分比
    double accPerc_joint_test = 2;              //设置加速度百分比
    double decPerc_joint_test = 2;              //设置减速度百分比
    double jerkPerc_joint_test = 2;             //设置雅可比速度百分比
    std::deque<double> trajectory_joint;

    //计算关节插补
    g_general_6s->move_joint_interp(target_point_joint,
                                    origin_point_joint_test, vel_current_joint_test, acc_current_joint_test, Ts_joint_test, velPerc_joint_test,
                                    accPerc_joint_test, decPerc_joint_test, jerkPerc_joint_test, trajectory_joint);

    if (!g_general_6s->get_power_on_status())
    {
        //判断使能状态
        g_general_6s->power_on(); //开启使能
    }
    sleep(2);
    //插补轨迹写入运动队列
    g_general_6s->set_angle_deque(trajectory_joint); //设置运动轨迹

    while (g_general_6s->get_power_on_status()) //循环检测使能状态
    {
        if (g_general_6s->get_angle_deque().empty() && g_general_6s->get_power_on_status())
        {
            g_general_6s->power_off(); //关闭使能
        }                              //判断运动状态
        usleep(100);
    }

    for (int i = 0; i < 6; i++)
    {
        cout << "joint" << i << "--" << g_general_6s->get_actual_position(i) << endl;
    }
    robot_is_moving = false;
}

void joint_zero()
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    VectorXd target_point_joint(6);      //目标位置,角度制
    VectorXd origin_point_joint_test(6); //初始位置,角度制
    VectorXd vel_current_joint_test(6);  //当前速度,角度制
    VectorXd acc_current_joint_test(6);  //当前加速度,角度制
    double pos_cur_ang[6];               //当前位置角度值,角度制

    for (int i = 0; i < 6; i++)
    {
        pos_cur_ang[i] = g_general_6s->get_actual_position(i); //获取当前位置角度值
        origin_point_joint_test(i) = pos_cur_ang[i];           //当前位置作为起始位置
        cout << "joint" << i << "--" << pos_cur_ang[i] << endl;
    }

    // 机器人标准初始姿态位置(0位置)
    for (int i = 0; i < 6; i++)
    {
        target_point_joint(i) = joint_deg(i, 0);
        // cout << joint_deg(i, 0) << endl;
    }

    vel_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前速度
    acc_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前加速度
    double Ts_joint_test = 0.001;               //设置运动周期
    double velPerc_joint_test = 2;              //设置速度百分比
    double accPerc_joint_test = 2;              //设置加速度百分比
    double decPerc_joint_test = 2;              //设置减速度百分比
    double jerkPerc_joint_test = 2;             //设置雅可比速度百分比
    std::deque<double> trajectory_joint;

    //计算关节插补
    g_general_6s->move_joint_interp(target_point_joint,
                                    origin_point_joint_test, vel_current_joint_test, acc_current_joint_test, Ts_joint_test, velPerc_joint_test,
                                    accPerc_joint_test, decPerc_joint_test, jerkPerc_joint_test, trajectory_joint);

    if (!g_general_6s->get_power_on_status())
    {
        //判断使能状态
        g_general_6s->power_on(); //开启使能
    }
    sleep(2);
    //插补轨迹写入运动队列
    g_general_6s->set_angle_deque(trajectory_joint); //设置运动轨迹

    while (g_general_6s->get_power_on_status()) //循环检测使能状态
    {
        if (g_general_6s->get_angle_deque().empty() && g_general_6s->get_power_on_status())
        {
            g_general_6s->power_off(); //关闭使能
        }                              //判断运动状态
        usleep(100);
    }
    cout << "joint zero end,current joint reg :" << endl;
    cout << "current joint reg :" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "joint" << i << "--" << g_general_6s->get_actual_position(i) << endl;
    }

    robot_is_moving = false;
}

void app_exit(void)
{
    exit(0);
}

void hello(void)
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    cout << "hello" << endl;
    Pdo_value pdo_value;
    pdo_value.int_value = 0x2000;
    g_general_6s->set_pdo_value(0x7011, 6, pdo_value);
    sleep(1);
    pdo_value.int_value = 0x0;
    g_general_6s->set_pdo_value(0x7011, 6, pdo_value);

    robot_is_moving = false;
}

void print_joints_deg(void)
{
    cout << "执行任务:print_joints_deg\n" << endl;
    cout << "各轴初始角度\n" << endl;
    for (int i=0; i<6; i++)
    {
        double actual_position;
        actual_position = g_general_6s->get_actual_position(i);
        printf("第%d轴原始角度为: ",i+1);
        printf("%f",actual_position);
        printf("\n");
    }
   
}

void print_joints_zero_deg(void)
{
    cout << "执行任务:print_joints_zero_deg\n" << endl;
    cout << "各轴调零角度\n" << endl;
    for (int i=0; i<6; i++)
    {
        double actual_position = g_general_6s->get_actual_position(i);
        double actual_bias = joint_bias[i];
        double actual_direction = joint_direction[i];
        double zero_deg = (actual_position-actual_bias)*actual_direction;
        printf("第%d轴调零角度为: ",i+1);
        printf("%f",zero_deg);
        printf("\n");
    }
}

void print_joints_effort(void)
{
    cout << "执行任务:print_joints_effort\n" << endl;
    cout << "各轴力矩\n" << endl;
    for (int i=0; i<6; i++)
    {
        double actual_position = g_general_6s->get_actual_position(i);
        double actual_rated_torque = joints_rated_torque[i];
        double actual_reduce_ratio = joints_reduce_ratio[i];
        double actual_torque = actual_position * actual_rated_torque * actual_reduce_ratio /1000.0/1000.0 ;
        printf("第%d轴力矩为: ",i+1);
        printf("%f",actual_torque);
        printf("\n");
    }
}

void print_joints_velocity(void)
{

    
    clock_t end_time=clock();
    cout << "执行任务:print_joints_velocity\n" << endl;
    cout << "各轴速度\n" << endl;
    for (int i=0; i<6; i++)
    {
        auto start = system_clock::now();
        double actual_position_1 = g_general_6s->get_actual_position(i);
        sleep(0.5);
        auto end   = system_clock::now();
        double actual_position_2 = g_general_6s->get_actual_position(i);
        auto duration = duration_cast<microseconds>(end - start);
        double period = double(duration.count());
        double actual_velocity = (actual_position_2 - actual_position_1) / period;
        printf("第%d轴速度为: ",i+1);
        printf("%f",actual_velocity);
        printf("\n");
    }
}

void joint_move_test(void)
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }
    int continue_flag = 1;
    while(continue_flag)
    {
        char joint_info[255];
        cout << "输入1-6, 使目标轴逆时针旋转10度" << endl;
        cin >> joint_info;
        char joint_axis_str[2];
        char joint_deg_str[255];
        // 读取关节轴
        joint_axis_str[0] = joint_info[0];
        joint_axis_str[1] = '\0';

        int axis = atoi(joint_axis_str);
        int deg = 10;

        cout << "使如下轴逆时针旋转10度：" << endl;
        cout << "轴:" << axis << endl;
        cout << "是否确定？yes|no" << endl;
        cin >> joint_info;
        if (0 != strcmp(joint_info, "yes"))
        {
            cout << "取消转动操作，程序退出";
            robot_is_moving = false;
            return;
        }

        VectorXd target_point_joint(6);      //目标位置,角度制
        VectorXd origin_point_joint_test(6); //初始位置,角度制
        VectorXd vel_current_joint_test(6);  //当前速度,角度制
        VectorXd acc_current_joint_test(6);  //当前加速度,角度制
        double pos_cur_ang[6];               //当前位置角度值,角度制

        for (int i = 0; i < 6; i++)
        {
            pos_cur_ang[i] = g_general_6s->get_actual_position(i); //获取当前位置角度值
            origin_point_joint_test(i) = pos_cur_ang[i];           //当前位置作为起始位置
            cout << "joint" << i << "--" << pos_cur_ang[i] << endl;
        }

        // 机器人标准初始姿态位置(0位置)
        for (int i = 0; i < 6; i++)
        {
            if (axis == i)
            {
                target_point_joint(i) = joint_add(i, pos_cur_ang[i], deg);
            }
            else
            {
                target_point_joint(i) = pos_cur_ang[i];
            }
        }

        vel_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前速度
        acc_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前加速度
        double Ts_joint_test = 0.001;               //设置运动周期
        double velPerc_joint_test = 2;              //设置速度百分比
        double accPerc_joint_test = 2;              //设置加速度百分比
        double decPerc_joint_test = 2;              //设置减速度百分比
        double jerkPerc_joint_test = 2;             //设置雅可比速度百分比
        std::deque<double> trajectory_joint;

        //计算关节插补
        g_general_6s->move_joint_interp(target_point_joint,
                                        origin_point_joint_test, vel_current_joint_test, acc_current_joint_test, Ts_joint_test, velPerc_joint_test,
                                        accPerc_joint_test, decPerc_joint_test, jerkPerc_joint_test, trajectory_joint);

        if (!g_general_6s->get_power_on_status())
        {
            //判断使能状态
            g_general_6s->power_on(); //开启使能
        }
        sleep(2);
        //插补轨迹写入运动队列
        g_general_6s->set_angle_deque(trajectory_joint); //设置运动轨迹

        while (g_general_6s->get_power_on_status()) //循环检测使能状态
        {
            if (g_general_6s->get_angle_deque().empty() && g_general_6s->get_power_on_status())
            {
                g_general_6s->power_off(); //关闭使能
            }                              //判断运动状态
            usleep(100);
        }

        for (int i = 0; i < 6; i++)
        {
            cout << "joint" << i << "--" << g_general_6s->get_actual_position(i) << endl;
        }
        robot_is_moving = false;

        cout << "是否继续操作： yes/no" <<endl;
        char continue_info[3];
        cin >> continue_info;
        if (0 != strcmp(joint_info, "yes"))
        {
            continue_flag = 0;
            cout << "运动结束，机械臂归零" <<endl;
            joint_zero();
        }

    }
   
}

void test_grasp(void)
{

    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    cout << "输入1使夹爪关闭，输入0使夹爪打开" << endl;
    cout << "输入为：" <<endl;
    char graspvalue;
    Pdo_value pdo_value;
    cin >> graspvalue;
    if ('1'==graspvalue)
    {
    pdo_value.int_value = 0x200;
    g_general_6s->set_pdo_value(0x7011, 6, pdo_value);
    }
    else if ('0'==graspvalue)
    {
    pdo_value.int_value = 0x00;
    g_general_6s->set_pdo_value(0x7011, 6, pdo_value);
    }
    cout << 'test succese' << endl;
    robot_is_moving = false;
}

void joint_cmd_action()
{

    char control_str[100];
    map<string, FnPtr> func_map = 
    {
        // 测试
        {"hello", hello},
        // 程序退出
        {"exit", app_exit},
        // 机械臂归零，使用五次多项式插值
        {"zero", joint_zero},
        // 打印关节角度信息
        {"print_joints_deg", print_joints_deg},
        // 打印关节调零角度信息
        {"print_joints_zero_deg", print_joints_zero_deg},
        // 打印关节角度信息
        {"print_joints_effort", print_joints_effort},
        // 打印关节角度信息
        {"print_joints_velocity", print_joints_velocity},
        // 单关节移动
        {"joint_move", joint_move},
        // 关节移动测试
        {"joint_move_test", joint_move_test},
        // io口测试
        {"test_grasp", test_grasp}
    };

    while (true)
    {
        cout << "请输入控制字：" << endl;
        cin >> control_str;
        if (func_map.count(control_str) == 0)
        {
            cout << "目标控制字不存在，请输入有效的控制字：" << endl;
            continue;
        }
        cout << "执行任务：" << control_str << endl;
        func_map[control_str]();
    }
}

void RobotFunc()
{
    /*   设置DH参数    */
    DH_param dh_example;
    dh_example.a[0] = 0;
    dh_example.a[1] = 295;
    dh_example.a[2] = 37;
    dh_example.a[3] = 0;
    dh_example.a[4] = 0;
    dh_example.a[5] = 0;
    dh_example.alpha[0] = M_PI * 90 / 180;
    dh_example.alpha[1] = M_PI * 0 / 180;
    dh_example.alpha[2] = M_PI * 90 / 180;
    dh_example.alpha[3] = M_PI * 90 / 180;
    dh_example.alpha[4] = M_PI * -90 / 180;
    dh_example.alpha[5] = M_PI * 0 / 180;
    dh_example.d[0] = 367.5;
    dh_example.d[1] = 0;
    dh_example.d[2] = 0;
    dh_example.d[3] = 295.5;
    dh_example.d[4] = 0;
    dh_example.d[5] = 78.5;
    dh_example.theta[0] = M_PI * 0 / 180;
    dh_example.theta[1] = M_PI * 90 / 180;
    dh_example.theta[2] = M_PI * 0 / 180;
    dh_example.theta[3] = M_PI * 0 / 180;
    dh_example.theta[4] = M_PI * 90 / 180;
    dh_example.theta[5] = M_PI * 0 / 180;
    g_general_6s->set_DH_param(dh_example);
    /*   设置笛卡尔参数  */
    Decare_Para decare;
    decare.maxacc = 3;
    decare.maxdec = -3;
    decare.maxjerk = 10000;
    decare.maxvel = 1000;
    g_general_6s->set_decare_param(decare);
    /*   设置电机参数  */
    Motor_Param motor_pa;
    motor_pa.encoder.reducRatio[0] = 81;
    motor_pa.encoder.reducRatio[1] = 101;
    motor_pa.encoder.reducRatio[2] = 63.462;
    motor_pa.encoder.reducRatio[3] = 68.966;
    motor_pa.encoder.reducRatio[4] = 81.25;
    motor_pa.encoder.reducRatio[5] = 40.625;
    motor_pa.encoder.singleTurnEncoder[0] = 533.822937;
    motor_pa.encoder.singleTurnEncoder[1] = 42.138062;
    motor_pa.encoder.singleTurnEncoder[2] = -0.302124;
    motor_pa.encoder.singleTurnEncoder[3] = -0.129089;
    motor_pa.encoder.singleTurnEncoder[4] = -976.308289;
    motor_pa.encoder.singleTurnEncoder[5] = -0.065918;
    for (int i = 0; i < 6; i++)
    {
        motor_pa.encoder.direction[i] = 1; //相对主轴方向
        motor_pa.encoder.deviation[i] = 0;
        motor_pa.encoder.encoderResolution[i] = 17;
        motor_pa.maxAcc[i] = 2.99;
        motor_pa.maxDecel[i] = -2.99;
        motor_pa.maxRotSpeed[i] = 3000;
        motor_pa.RatedVel[i] = motor_pa.maxRotSpeed[i] * 6 / motor_pa.encoder.reducRatio[i];
        motor_pa.DeRatedVel[i] = -motor_pa.RatedVel[i];
    }
    g_general_6s->set_motor_param(motor_pa);

    /*    主要功能块    */
    joint_cmd_action();
}

int start_controller()
{
    /*  初始化robot指针  */
    printf("/*  初始化robot指针  */\n");
    g_general_6s = new General_6S();
    printf("g_general_6s->register_cycle_run();\n");
    g_general_6s->register_cycle_run();
    /*  开启主功能块  */
    printf("/*  开启主功能块  */\n");
    RobotFunc();
    return 0;
}

int main(int argc, char *argv[])
{
    // char *argv_param[3];
    // char param_1[20] = " -f";
    // char param_2[20] = " eni/cooldriver.xml";

    // argv_param[0] = argv[0];
    // argv_param[1] = param_1;
    // argv_param[2] = param_2;

    /*  启动主站  */
    Robot::add_pdo(0x7011, pdo_object_type::int_);

    printf("/*  启动主站  */\n");
    start_ecm(3, argv);
    /*  启动控制器程序  */
    printf("/*  启动控制器程序  */\n");
    start_controller();
    /*  循环显示  */
    printf("/*  循环显示  */\n");
    loop_display();
}
