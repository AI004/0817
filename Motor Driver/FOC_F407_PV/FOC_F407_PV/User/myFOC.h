#ifndef __myFOC_H
#define __myFOC_H

#include <stdint.h>
#include "arm_math.h"
#include "tim.h"
#include "Serial.h"
#include "math_utils.h"
#include "ADC.h"


void PWM_Init(void);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
void Set_PWM(float _CCR1, float _CCR2, float _CCR3);
void adc_uesr_Init(void);
void adc_calibrate_offset(void);
void Id_Iq_calculate(float Ia,float Ib,float el_Angle);


#endif
