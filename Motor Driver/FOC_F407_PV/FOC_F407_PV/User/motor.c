#include "motor.h"
#include "math_utils.h"
#include "AS5600.h"
#include "PID.h"
#include "Serial.h"

///**********************  开环速度控制  ********************************************/
//// 开环运行测试1，模拟电角度
//void OpenVelocity1(float target)
//{
//    const float voltageLimit = 0.5f;    // 开环电压限制
//    static float _estimateAngle = 0.0f; // 开环虚拟机械角度
//    const float deltaT = 0.00625f;      // 开环运行时间间隔
//    _estimateAngle = _normalizeAngle((_estimateAngle + target * deltaT) * 7);

//}

//// 开环速度控制测试，使用MT6701反馈的电角度
//void OpenVelocity2(float Uq)
//{
//    float el_Angle = AS5600_GetElectricalAngle();

//}
// /************  闭环电流控制  **********************************************/
// 电流环PID参数配置
//PID_Typedef current_PID;
//float set_current = 0.0;
//float actual_current = 0.0;
//float current_kp = 0.1f;
//float current_ki = 0.01f;
//float current_kd = 0.0f;
//// 电流环PID参数设置
//void current_PID_Config(void)
//{
//    current_PID.Kp = current_kp;
//    current_PID.Ki = current_ki;
//    current_PID.Kd = current_kd;
//    current_PID.Ts = 0.001f;
//    current_PID.Umax = 5.0f;
//    PID_Init(&current_PID, current_PID.Kp, current_PID.Ki, current_PID.Kd, current_PID.Umax, current_PID.Ts);
//}

//// 电流环PID控制
//void close_Current_Control(void)
//{
//    actual_current = AS5600_GetCurrent(); // 实际电流
//    LowPassFilter(&actual_current);             // 对电流进行低通滤波

//    float Iq = PIDCalculate(&current_PID, set_current, actual_current); // 电流环PID控制

//    float el_Angle = AS5600_GetElectricalAngle();
//    setPhaseVoltage(Iq, 0.0f, el_Angle);
//}
//// 串口输出目标电流，实电流
//void close_Current_LOG(void)
//{
//    FOC_log("[SetValue,ActualValue]:%f,%f\r\n", set_current, actual_current);
//}
// /************  闭环速度控制  **********************************************/
float Set_Velocity = 50.0f; // 目标转速
float actual_Velocity;      // 实际转速
extern float Current_Ia;           //Ia电流
extern float Current_Ib;           //Ib电流
extern float current_Iq;           //Iq电流
extern float current_Id;           //Id电流

float velocity_kp = 0.1f;
float velocity_ki = 0.1f;
PID_Typedef velocity_PID; // 速度环PID参数配置
PID_Typedef current_Iq_PID;  // 电流环PID参数配置
PID_Typedef current_Id_PID;  // 电流环PID参数配置

// 闭环速度PID控制参数设置
void velocity_PID_Config(void)
{
    velocity_PID.Kp = 0.1;
    velocity_PID.Ki = 0.1;
    velocity_PID.Kd = 0.0f;
    velocity_PID.Ts = 0.001f;
    velocity_PID.Umax = 50.0f;
    PID_Init(&velocity_PID, velocity_PID.Kp, velocity_PID.Ki, velocity_PID.Kd, velocity_PID.Umax, velocity_PID.Ts);
}

// 速度环PID控制
void close_Velocity_Control(void)
{
    actual_Velocity = AS5600_GetVelocity(); // 实际转速
    LowPassFilter(actual_Velocity);        // 对转速进行低通滤波
	
    float Uq = PIDCalculate(&velocity_PID, Set_Velocity, actual_Velocity); // 速度环PID控制

    float el_Angle = AS5600_GetElectricalAngle();
		setPhaseVoltage(Uq, 0.0f, el_Angle);
}

// 串口输出目标转速，实际转速
void close_Velocity_LOG(void)
{
    FOC_log("[SetValue,ActualValue]:%f,%f\r\n", Set_Velocity, actual_Velocity);
}

// /************闭环位置控制**********************************************/
float Set_Angle = 3.1415926f;
float actual_Angle;
PID_Typedef angle_PID;
float angle_kp = 200.0f;

// 位置环PID参数配置
void angle_PID_Config(void)
{
    angle_PID.Kp = 200;
    angle_PID.Ki = 0.0f;
    angle_PID.Kd = 0.0f;
    angle_PID.Umax = 50.0f; // 角度环的响应速度
    angle_PID.Ts = 0.001f;
    PID_Init(&angle_PID, angle_PID.Kp, angle_PID.Ki, angle_PID.Kd, angle_PID.Umax, angle_PID.Ts);
}

// 位置环PID控制
void close_Angle_Control(void)
{
    static int i=0;
    actual_Angle = AS5600_GetRawAngle(); // 实际机械角度
    i++;
    Set_Angle = 3;//+2*sin(2*3.1415926*i/1000);
    float set_Velocity = PIDCalculate(&angle_PID, Set_Angle, actual_Angle); // 角度换PID控制，输出转速

    float actual_Velocity = AS5600_GetVelocity(); // 实际转速
    LowPassFilter(actual_Velocity);

    float Uq = PIDCalculate(&velocity_PID, set_Velocity, actual_Velocity);

    float el_Angle = AS5600_GetElectricalAngle();
    setPhaseVoltage(Uq, 0.0f, el_Angle);
}

// 位置环串口输出目标角度、实际角度
void close_Angle_LOG(void)
{
//    FOC_log("[SetValue,ActualValue]:%f,%f\r\n", Set_Angle, actual_Angle);
//    FOC_log("%f,%f,%f,%f\r\n", Set_Angle, actual_Angle,Current_Ia, Current_Ib);
  static int i=0;
  i++;
  if(i%10000==0){
    FOC_log("%f,%f\r\n", Current_Ia, Current_Ib);
  }
}

// /************闭环电流控制**********************************************/

// 闭环电流PID控制参数设置
void current_Iq_PID_Config(void)
{
    current_Iq_PID.Kp = 0.01f;
    current_Iq_PID.Ki = 0.1f;
    current_Iq_PID.Kd = 0.0f;
    current_Iq_PID.Ts = 0.001f;
    current_Iq_PID.Umax = 3.0f;
    PID_Init(&current_Iq_PID, current_Iq_PID.Kp, current_Iq_PID.Ki, current_Iq_PID.Kd, current_Iq_PID.Umax, current_Iq_PID.Ts);
}
// 闭环电流PID控制参数设置
void current_Id_PID_Config(void)
{
    current_Id_PID.Kp = 0;
    current_Id_PID.Ki = 0;
    current_Id_PID.Kd = 0;
    current_Id_PID.Ts = 0.001f;
    current_Id_PID.Umax = 0;
    PID_Init(&current_Id_PID, current_Id_PID.Kp, current_Id_PID.Ki, current_Id_PID.Kd, current_Id_PID.Umax, current_Id_PID.Ts);
}
float Iq_set;
float Id_set;

// 闭环电流控制
void close_Current_Control(void)
{
    float el_Angle =AS5600_GetElectricalAngle(); // 电角度

    Id_Iq_calculate(Current_Ia,Current_Ib,el_Angle); // 计算Iq、Id的电流

//    LowPassFilter(current_Iq);
//    LowPassFilter(current_Id);

//    float Uq = PIDCalculate(&current_Iq_PID, Iq_set, current_Iq);
//    float Ud = PIDCalculate(&current_Id_PID, Id_set, current_Id);

//    setPhaseVoltage(Uq, Ud, el_Angle);
}



















