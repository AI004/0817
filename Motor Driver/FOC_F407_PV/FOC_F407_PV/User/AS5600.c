#include "as5600.h"
#include "IIC_1.h"

#define AS5600ADDR (0x6cU)
#define GYROCR (0x0eU)


const uint8_t pole_pairs = 7; // 极对数
float zeroElectricAngleOffset = 0.0f;


int rotationCount = 0;  // 旋转过的圈数
int rotationCount_Last; // 上一次循环时转过的圈数

// 获取AS5600原始数据
uint16_t AS5600_GetRawData(void)
{
  uint16_t rawData;
//  uint8_t txData[2] = {0};
//  uint16_t timeOut = 200;
//  HAL_I2C_Mem_Read(&hi2c2, AS5600ADDR, GYROCR, I2C_MEMADD_SIZE_8BIT, txData, 2, timeOut);
//  rawData = (txData[0] << 8) | txData[1] ;
  
  
  rawData = AS5600_ReadTwoByte();
  
  
  return rawData ; // 取高14位的角度数据
}

// 获得原始角度，无圈数累加
float AS5600_GetRawAngle(void)
{
  uint16_t rawData = AS5600_GetRawData();
  return (float)rawData / 4096.0f * _2PI;
}

// 获得转过的总角度，有圈数累加
float AS5600_GetFullAngle(void)
{
  static float angle_Last = 0.0f;     // 上次的轴角度，范围0~6.28
  float angle = AS5600_GetRawAngle(); // 当前角度，范围0~6.28
  float deltaAngle = angle - angle_Last;

  // 计算旋转的总圈数
  // 通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。

  if (fabsf(deltaAngle) > (0.8f * 6.28318530718f))
  {
    rotationCount += (deltaAngle > 0) ? -1 : 1; // 圈数计算
    rotationCount_Last = rotationCount;
  }

  angle_Last = angle;
  return rotationCount * 6.28318530718f + angle_Last; // 转过的圈数 * 2pi + 未转满一圈的角度值
}

// 计算转速
float AS5600_GetVelocity(void)
{
  static float full_Angle_Last = 0.0f; // 记录上次转过的总角度
  float full_Angle = AS5600_GetFullAngle();
  float delta_Angle = (rotationCount - rotationCount_Last) * _2PI + (full_Angle - full_Angle_Last);
  float vel = delta_Angle * 1000.0f; // Ts = 1ms

  // 更新变量值
  full_Angle_Last = full_Angle;
  return vel;
}

// 获得电角度
float AS5600_GetElectricalAngle(void)
{
	return _normalizeAngle(pole_pairs *AS5600_GetRawAngle() - zeroElectricAngleOffset);
}

// 电角度零位校准
void Align_Sensor(void)
{
  int i = 0;
  // 电角度零位标定方式1：
//  setPhaseVoltage(3.0f, 0.0f, _3PI_2);
//  HAL_Delay(1000);
//  zeroElectricAngleOffset = AS5600_GetElectricalAngle(); // 测量电角度零位偏差
//  setPhaseVoltage(0, 0, _3PI_2);
//  float zero_elAngle = AS5600_GetElectricalAngle(); // 观察电角度是否为0

  // 电角度零位标定方式2：
  for(i=0;i<200;i++)
  {
    setPhaseVoltage(0.0f, 1.0f, 0.0f);
  }
//  HAL_Delay(100);
  zeroElectricAngleOffset = AS5600_GetElectricalAngle(); // 测量电角度零位偏差
  float zero_elAngle = AS5600_GetElectricalAngle();      // 观察电角度是否为0


}

// MT6701编码器数据输出
void AS5600_log(void)
{
  float angle = AS5600_GetRawAngle();
  float fullAngle = AS5600_GetFullAngle();
  float velocity = AS5600_GetVelocity();
  float ElectricalAngle = AS5600_GetElectricalAngle();

//    FOC_log("[angle,fullAngle,vel]:%f,%f,%f\r\n", angle, fullAngle, velocity);
  FOC_log("[angle,fullAngle,ElectricalAngle]:%f,%f,%f\r\n", angle, fullAngle, ElectricalAngle);
}
