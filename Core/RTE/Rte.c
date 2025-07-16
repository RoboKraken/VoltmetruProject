/*
 * Rte.c
 *
 *  Created on: Jul 15, 2025
 *      Author: Robert
 */
#include "main.h"


uint16_t voltReadRaw=1;
uint16_t voltRead=0;
uint8_t displayMode=0;
volatile uint16_t adc_buffer[2];
