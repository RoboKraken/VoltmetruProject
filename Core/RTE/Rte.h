/*
 * Rte.h
 *
 *  Created on: Jul 15, 2025
 *      Author: Robert
 */

#ifndef RTE_RTE_H_
#define RTE_RTE_H_

extern uint16_t voltReadRaw;
extern uint16_t buttonReadRaw;
extern uint16_t voltRead;
extern uint16_t buttonRead[2];
extern uint8_t displayMode;
extern uint8_t displayModeMax;
extern volatile uint16_t adc_buffer[2];

#endif /* RTE_RTE_H_ */
