/*
 * Rte.c
 *
 *  Created on: Jul 15, 2025
 *      Author: Robert
 */
#include "main.h"


uint16_t voltReadRaw=1;
uint16_t buttonReadRaw=1;
uint16_t voltRead=0;
uint16_t buttonRead[2];
uint8_t displayMode=1;
uint8_t displayModeMax=1;

uint8_t fontMode=1;

uint8_t buttonState = 0;
uint8_t buttonStatePrev = 0;
uint32_t buttonDebounceTimer = 0;
uint8_t buttonTransitionFlag = 0;

uint32_t displayModeChangeTime = 0;
uint8_t showDisplayModeOverlay = 0;
volatile uint16_t adc_buffer[2];
