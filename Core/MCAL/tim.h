/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

typedef enum {
	FREQ_1HZ,
	FREQ_10HZ,
	FREQ_20HZ,
    FREQ_50HZ,
    FREQ_100HZ,
	FREQ_200HZ,
	FREQ_500HZ,
	FREQ_1KHZ,
	FREQ_2KHZ,
	FREQ_5KHZ,
	FREQ_10KHZ,
	FREQ_20KHZ,
	FREQ_50KHZ,
    FREQ_MODE_MAX
} FrequencyMode;

extern FrequencyMode currentFreqMode;

extern TIM_HandleTypeDef htim2;

extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);

/* USER CODE BEGIN Prototypes */
extern const uint32_t tim3_arr_values[FREQ_MODE_MAX];

void update_tim3_frequency(FrequencyMode mode);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

