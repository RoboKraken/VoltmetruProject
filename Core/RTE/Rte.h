/*
 * Rte.h
 *
 *  Created on: Jul 15, 2025
 *      Author: Robert
 */

#ifndef RTE_RTE_H_
#define RTE_RTE_H_
#define ADC_BUFFER_SIZE 256

extern uint16_t voltReadRaw;
extern uint16_t buttonReadRaw;
extern uint16_t voltRead;
extern uint16_t buttonRead[2];
extern uint8_t displayMode;
extern uint8_t displayModeMax;
extern volatile uint16_t adc_buffer0[];
extern volatile uint16_t adc_buffer1[];
extern volatile uint8_t active_buffer_id;

extern uint8_t fontMode;
extern uint8_t fontModeMax;

extern uint8_t buttonState;
extern uint8_t buttonStatePrev;
extern uint32_t buttonDebounceTimer;
extern uint8_t buttonTransitionFlag;

extern uint32_t displayModeChangeTime;
extern uint8_t showDisplayModeOverlay;

extern uint32_t os_task_overrun_count[8];
extern uint32_t os_task_overrun_time;
extern uint32_t os_debug_drawing_time;

extern uint16_t oscilloscopeBuffer[];
extern uint16_t oscilloscopeTrigger;
extern uint16_t testOscilloscope;
extern uint16_t oscilloscopeTriggerRaw;
extern uint16_t oscilloscopeTriggerHyst;
extern uint16_t oscilloscopeTriggerIndex;
#endif /* RTE_RTE_H_ */
