/*
 * tim2_microsecond_counter.h
 *
 *  Created on: Jul 22, 2025
 *      Author: Robert
 */

#ifndef MCAL_TIM2_MICROSECOND_COUNTER_H_
#define MCAL_TIM2_MICROSECOND_COUNTER_H_

extern void TIM2_Start_1MHz(void);
extern uint32_t HAL_GetTick_us(void);
extern void HAL_Delay_us(uint32_t us);

#endif /* MCAL_TIM2_MICROSECOND_COUNTER_H_ */
