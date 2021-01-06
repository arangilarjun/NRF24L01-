#include "stm32f0xx.h"

void delay_init(void) {
	TIM_TimeBaseInitTypeDef time_base;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseStructInit(&time_base);
	TIM_TimeBaseInit(TIM1, &time_base);
	TIM_ARRPreloadConfig(TIM1, DISABLE);
}
void delay_ms(uint32_t ms) {
	TIM_Cmd(TIM1, ENABLE);
	TIM_SetCounter(TIM1, ms);

	while (TIM_GetCounter(TIM1) != 0)
		;
	TIM_Cmd(TIM1, DISABLE);

}

