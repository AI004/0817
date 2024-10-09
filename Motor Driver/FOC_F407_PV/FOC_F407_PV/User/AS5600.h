#ifndef __AS5600_H
#define __AS5600_H

#include "i2c.h"
#include "Serial.h"
#include <stdint.h>
#include "math_utils.h"
#include "myFOC.h"

void AS5600_log(void);   // MT6701编码器数据输出
void Align_Sensor(void); // 电角度零位校准

float AS5600_GetRawAngle(void);        // 机械角度
float AS5600_GetElectricalAngle(void); // 电角度
float AS5600_GetVelocity(void);        // 转速
#endif
