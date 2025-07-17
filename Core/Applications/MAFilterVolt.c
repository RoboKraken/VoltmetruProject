/*
 * MovingAverageFilter.c
 *
 *  Created on: Jul 15, 2025
 *      Author: Robert
 */
#include <stdint.h>

static uint16_t MAVectorVolt[10];

static uint16_t filterVolt(uint16_t voltIn){
	uint16_t media=voltIn;
	for(uint8_t i=0;i<9;i++){
		media+=MAVectorVolt[i+1];
		MAVectorVolt[i]=MAVectorVolt[i+1];
	}
	MAVectorVolt[9]=voltIn;
	media/=10;
	return media;
}

