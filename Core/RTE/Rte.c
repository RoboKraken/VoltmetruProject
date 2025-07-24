/*
 * Rte.c
 *
 *  Created on: Jul 15, 2025
 *      Author: Robert
 */
#include "main.h"
#include "Rte.h"

uint16_t voltReadRaw=1;
uint16_t buttonReadRaw=1;
uint16_t voltRead=0;
uint16_t buttonRead[2];
uint8_t displayMode=2;
uint8_t displayModeMax=2;

uint8_t fontModeMax=1;
uint8_t fontMode=0;

uint8_t buttonState = 0;
uint8_t buttonStatePrev = 0;
uint32_t buttonDebounceTimer = 0;
uint8_t buttonTransitionFlag = 0;

uint32_t displayModeChangeTime = 0;
uint8_t showDisplayModeOverlay = 0;
uint32_t os_task_overrun_count[8] = {0};
uint32_t os_task_overrun_time=0; //Cat de mult a durat taskul care a rulat prea mult
uint32_t os_debug_drawing_time=0;

volatile uint16_t adc_buffer0[ADC_BUFFER_SIZE];
volatile uint16_t adc_buffer1[ADC_BUFFER_SIZE];
volatile uint8_t active_buffer_id = 0;
uint16_t oscilloscopeBuffer[ADC_BUFFER_SIZE];
uint16_t oscilloscopeTrigger=200;
uint16_t testOscilloscope=0;
uint16_t oscilloscopeTriggerRaw = 2048; 
uint16_t oscilloscopeTriggerHyst = 10;
uint16_t oscilloscopeTriggerIndex=1;
uint8_t adc_buffer_ready=0;
uint8_t trigger_found = 0;
uint8_t trigger_active=1;

uint8_t menu_active=0;
uint8_t menu_select=0;
