#include <tim.h>
#include <delay_us.h>

// DMA模式
void delay_us(uint32_t nus)
{

    uint16_t  differ = 0xffff-nus-5;
    //设置定时器2的技术初始值
  __HAL_TIM_SetCounter(&htim2,differ);
  //开启定时器
  HAL_TIM_Base_Start(&htim2);

  while( differ<0xffff-5)
    {
        differ = __HAL_TIM_GetCounter(&htim2);
    };
 //关闭定时器
  HAL_TIM_Base_Stop(&htim2);
}
