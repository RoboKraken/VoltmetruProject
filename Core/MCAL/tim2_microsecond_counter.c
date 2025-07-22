/*
 * tim2_microsecond_counter.c
 *
 *  Created on: Jul 22, 2025
 *      Author: Robert
 */

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_tim.h"

/*void TIM2_Start_1MHz(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM2->PSC = (SystemCoreClock / 1000000U) - 1U;   // 48-1 → ÷48
    TIM2->ARR = 0xFFFFFFFF;                          // free-run
    TIM2->CR1 = TIM_CR1_CEN;
}*/

uint32_t HAL_GetTick_us(void)
{
    return TIM2->CNT;          // 1-µs resolution
}

void HAL_Delay_us(uint32_t us)
{
    uint32_t start = micros();
    while ((micros() - start) < us);
}
