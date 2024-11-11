#ifndef TI5MOVE_H
#define TI5MOVE_H

#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <thread>
#include "can/SingleCaninterface.h"
#include "can/motortypehelper.h"
#include <vector>
#include "tool.h"
#include "Ti5LOGIC.h"
#include "Ti5BASIC.h"
#include <time.h>

//#define USLEEPTIME 3000 // 单条指令通信间隔（微秒）
#define RCV_Size (IDNUM + 2) * 4

extern float AG;    // 启停时变速的采样间距（秒），须>=4*USLEEPTIME
extern float scale; // 电机内圈与外圈的速度比
extern float n2p;   // 内圈转速到步速的转化系数
extern float mvtime;
extern float j2p;                // 电机外圈角度到内圈步数的转化
extern class humanoidLeftArm TH; // 机械臂类（正逆运动解算器的接口
extern float min_time;
extern float NMAX; // 所有电机内核最大转速值（(NMAX/100)圈/秒）
extern float jstep;
extern bool jstp;
extern uint8_t canidList[IDNUM];
extern int clientFd;
extern float nplL[4][4];  // add 用于linear_move函数机械臂直线运动
extern float angl[IDNUM]; // bais[6]:实际位置相较于理论位置的偏置

extern int lmsg;
extern float msg[1024];

extern char Info_Str[100]; // 定义一个字符数组用于存储 flag 的字符串表示


extern "C"
{ // 添加：extern C

    /*机械臂关节运动
		参数：
			goal_j：目标关节角
		返回值：无*/
	void joint_to_move(float *goal_j);
    bool interrupt_joint_to_move(float *goal_j,std::atomic<bool>& stopMovement);//有中断功能
    void setGetCSP_joint_to_move(float *goal_j,int32_t *positionList);

    /*机械臂关节运动，同时获取当前位置
        参数：
            goal_j：目标关节角
            CUrrentJointPosition：存储当前位置
        返回值：无
    */
    void GetP_joint_to_move(float *goal_j,float *CUrrentJointPosition);

	/*pos运动
    参数：
        pos：目标位置
        value：dim的值
        dim：0~2 代表x,y,z
        absolute：true的时候是绝对位置（以胸部原点位置），false的时候是相对位置（以当前点胳膊轴位置）
    返回值：无
    */
    void pos_to_move(float *pos,float value,int dim,bool absolute);

    /*获取当前角度
    参数：
        goal_j：存储角度的数组
    */
    void get_current_angle(float goal_j[7]);
    bool interrupt_get_current_angle(float goal_j[7],std::atomic<bool>& stopMovement);//有中断功能

    /*获取当前位姿
    参数：
        posz：存储位姿的数组
    */
    void get_current_pose(float posz[6]);

    /*机械臂刹车*/
    bool brake();

    /*机械臂是否停止运动*/
    bool IsBrake();

    // void write_value(float array[]);
    /*将数据记录下来写入文件
    参数：
        pj_flag：角度或者位姿标识,1为角度 ，0为坐标
        filename：存储文件名
        array[6]：被保存的值
    */
    void write_value(int pj_flag, string filename, float array[7]);

    /*机械臂回到原点*/
    void mechanical_arm_origin();

    void setn(int npL[IDNUM]);

    void movebyn(float npL[IDNUM]);

    void ACTmove(float *a, float *b, float T0);
    bool interrupt_joint_ACTmove(float *a, float *b, float T0,std::atomic<bool>& stopMovement); // 实际运动
	void setGetCSP_ACTmove(float *a, float *b, float T0,int32_t positionList[7]);

    void plan_move();
    void interrupt_plan_move(std::atomic<bool>& stopMovement);
    void setGetCSP_plan_move(int32_t *positionList);
    void GetP_plan_move(float *CUrrentJointPosition);

    void Upper_controller();

    /*没有解算器和正运动*/
    void set_motor_position(float crtj[IDNUM]);

    //右臂运动
    void RightArmMove(float *crtj);

} // 添加：extern C
#endif
