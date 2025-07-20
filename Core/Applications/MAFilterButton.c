/*
 * MAFilterButton.c
 *
 *  Created on: Jul 17, 2025
 *      Author: Robert
 */
#include <stdint.h>

static uint16_t MAVectorButton[10];

static uint16_t filterButton(uint16_t voltIn){
	uint16_t media=voltIn;
	for(uint8_t i=0;i<9;i++){
		media+=MAVectorButton[i+1];
		MAVectorButton[i]=MAVectorButton[i+1];
	}
	MAVectorButton[9]=voltIn;
	media/=10;

	//if(media>)
	return media;
}


