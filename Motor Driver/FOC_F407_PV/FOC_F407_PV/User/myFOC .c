#include "myFOC.h"

const float sqrt3 = 1.73205080756f;
const int PWM_ARR = 8000;		// PWM计数周期
const float Udc = 12.0f;      // 电机母线电压

float current_gain;           //电流增益
float shunt_Resistor = 0.01f; //采样电阻
float TP181_Gain = 50.0f;      //运放增益

float zero_offsetA,zero_offsetB;
float vref_voltage = 3.3f;   //偏置电压

float Current_Ia;           //Ia电流
float Current_Ib;           //Ib电流

float current_Iq;           //Iq电流
float current_Id;           //Id电流

// 使能TIMx的通道y
void PWM_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}

// 输入参数0.0f ~ 1.0f，输出3路PWM
void Set_PWM(float _CCR1, float _CCR2, float _CCR3)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, _CCR1 * PWM_ARR); // PE9
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, _CCR2 * PWM_ARR); // PE11
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, _CCR3 * PWM_ARR); // PE13
}


// FOC核心函数：输入Uq、Ud和电角度，输出三路PWM
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	Uq = _constraint(Uq, -5.0f, 5.0f);
	
	static float Ts = 1.0f;
	float Ta, Tb, Tc;
	float t1, t2, t3, t4, t5, t6, t7;
	float sum, k_svpwm;

	// Park逆变换
	float U_alpha = -Uq * arm_sin_f32(angle_el) + Ud * arm_cos_f32(angle_el);
	float U_beta = Uq * arm_cos_f32(angle_el) + Ud * arm_sin_f32(angle_el);

	// 扇区判断
	float K = sqrt3 * Ts / Udc; // SVPWM调制比
	float u1 = U_beta * K;
	float u2 = (0.8660254f * U_alpha - 0.5f * U_beta) * K; // sqrt(3)/2 = 0.8660254
	float u3 = (-0.8660254f * U_alpha - 0.5f * U_beta) * K;

	uint8_t sector = (u1 > 0.0f) + ((u2 > 0.0f) << 1) + ((u3 > 0.0f) << 2); // sector = A + 2B + 4C

	// 非零矢量和零矢量作用时间的计算
	switch (sector)
	{
	case 3: // 扇区1
		t4 = u2;
		t6 = u1;
		sum = t4 + t6;
		if (sum > Ts) // 过调制处理
		{
			k_svpwm = Ts / sum;
			t4 *= k_svpwm;
			t6 *= k_svpwm;
		}
		t7 = (Ts - t4 - t6) / 2.0f;
		Ta = t4 + t6 + t7;
		Tb = t6 + t7;
		Tc = t7;
		break;
	case 1: // 扇区2
		t2 = -u2;
		t6 = -u3;
		sum = t2 + t6;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t2 *= k_svpwm;
			t6 *= k_svpwm;
		}
		t7 = (Ts - t2 - t6) / 2.0f;
		Ta = t6 + t7;
		Tb = t2 + t6 + t7;
		Tc = t7;
		break;
	case 5: // 扇区3
		t2 = u1;
		t3 = u3;
		sum = t2 + t3;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t2 *= k_svpwm;
			t3 *= k_svpwm;
		}
		t7 = (Ts - t2 - t3) / 2.0f;
		Ta = t7;
		Tb = t2 + t3 + t7;
		Tc = t3 + t7;
		break;
	case 4: // 扇区4
		t1 = -u1;
		t3 = -u2;
		sum = t1 + t3;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t1 *= k_svpwm;
			t3 *= k_svpwm;
		}
		t7 = (Ts - t1 - t3) / 2.0f;
		Ta = t7;
		Tb = t3 + t7;
		Tc = t1 + t3 + t7;
		break;
	case 6: // 扇区5
		t1 = u3;
		t5 = u2;
		sum = t1 + t5;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t1 *= k_svpwm;
			t5 *= k_svpwm;
		}
		t7 = (Ts - t1 - t5) / 2.0f;
		Ta = t5 + t7;
		Tb = t7;
		Tc = t1 + t5 + t7;
		break;
	case 2: // 扇区6
		t4 = -u3;
		t5 = -u1;
		sum = t4 + t5;
		if (sum > Ts)
		{
			k_svpwm = Ts / sum;
			t4 *= k_svpwm;
			t5 *= k_svpwm;
		}
		t7 = (Ts - t4 - t5) / 2.0f;
		Ta = t4 + t5 + t7;
		Tb = t7;
		Tc = t5 + t7;
		break;
	default:
		break;
	}

//	 FOC_log("[Ta,Tb,Tc]:%f,%f,%f\r\n", Ta, Tb, Tc);
	Set_PWM(Ta, Tb, Tc); // 输出三路PWM，驱动无刷电机转动
}
// 注入组ADC配置初始化
void adc_uesr_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // 开启TIM1_CH4，用于触发ADC转换
    HAL_ADCEx_InjectedStart(&hadc1); // 启动注入模式的ADC转换
    current_gain = 1.0f / shunt_Resistor / TP181_Gain;
    adc_calibrate_offset(); // 计算ADC的偏置电压
}
// ADC采样偏置计算
void adc_calibrate_offset(void)
{
    uint16_t cnt = 1000;

    for (uint16_t i = 0; i < cnt; i++)
    {
        uint16_t adc1_value_Rank1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint16_t adc1_value_Rank2 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

        zero_offsetA += (float)adc1_value_Rank1 / 4095.0f * vref_voltage;
        zero_offsetB += (float)adc1_value_Rank2 / 4095.0f * vref_voltage;
    }

    zero_offsetA = zero_offsetA / cnt;
    zero_offsetB = zero_offsetB / cnt;


    HAL_ADCEx_InjectedStart_IT(&hadc1); // 开启ADC注入组中断
}
/* 注入组完成转换后的中断回调函数
 * 1.计算电流采样值
 * 2.获取转速
 * 3.获取电角度
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        // 获取注入组ADC1的转换值
        uint16_t adc1_value_Rank1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        uint16_t adc1_value_Rank2 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

        // 计算相电流
//        Current_Ia = ((float)adc1_value_Rank1 / 4095.0f * vref_voltage - zero_offsetA) ;
//        Current_Ib = ((float)adc1_value_Rank2 / 4095.0f * vref_voltage - zero_offsetB) ;
//        Current_Ia = ((float)adc1_value_Rank1 / 4095.0f * vref_voltage - 0) ;
//        Current_Ib = ((float)adc1_value_Rank2 / 4095.0f * vref_voltage - 0) ;
				Current_Ia = ((float)adc1_value_Rank1 / 4095.0f * vref_voltage-1.65f)/50.0f;
        Current_Ib = ((float)adc1_value_Rank2 / 4095.0f * vref_voltage-1.65f)/50.0f;
			  

    }
}
//计算Iq Id PARK变换
void Id_Iq_calculate(float Ia,float Ib,float el_Angle)
{
	current_Id =  Ia * arm_cos_f32(el_Angle) + Ib * arm_sin_f32(el_Angle);
	current_Iq = -Ia * arm_sin_f32(el_Angle) + Ib * arm_cos_f32(el_Angle);
}
