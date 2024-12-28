#include "main.h"

extern TIM_HandleTypeDef htim6;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == USER_BTN_Pin){
		HAL_TIM_Base_Start_IT(&htim6);
	}
}
