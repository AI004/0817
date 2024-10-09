#ifndef __MOTOR_H
#define __MOTOR_H

#include "myFOC.h"
// 函数声明
void OpenVelocity1(float target); // 开环运行测试，模拟电角度
void OpenVelocity2(float Uq);     // 开环转速测试，编码器反馈电角度，无转速控制

void current_PID_Config(void);
void close_Current_Control(void);
void close_Current_LOG(void);

void velocity_PID_Config(void);    // 闭环速度PID控制,参数设置
void close_Velocity_Control(void); // 速度环PID控制
void close_Velocity_LOG(void);     // 串口输出目标转速，实际转速

void angle_PID_Config(void);
void close_Angle_Control(void);
void close_Angle_LOG(void);

void current_Iq_PID_Config(void);
void current_Id_PID_Config(void);
void close_Current_Control(void);
	
#endif
