/*
 * MovingAverageFilter.c
 *
 *  Created on: Jul 15, 2025
 *      Author: Robert
 */
#include <stdint.h>

static uint16_t MAVector[10];

static uint16_t filter(uint16_t voltIn){
	uint16_t media=voltIn;
	for(uint8_t i=2;i<9;i++){
		media+=MAVector[i-1];
		MAVector[i]=MAVector[i-1];
	}
	MAVector[9]=voltIn;
	media/=10;
	return media;
}

