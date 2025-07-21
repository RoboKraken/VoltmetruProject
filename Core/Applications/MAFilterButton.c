/*
 * MAFilterButton.c
 *
 *  Created on: Jul 17, 2025
 *      Author: Robert
 */
#include <stdint.h>

static uint16_t MAVectorButton[4];

static uint16_t filterButton(uint16_t voltIn){
	uint16_t media=voltIn;
	for(uint8_t i=0;i<3;i++){
		media+=MAVectorButton[i+1];
		MAVectorButton[i]=MAVectorButton[i+1];
	}
	MAVectorButton[3]=voltIn;
	media/=4;

	//if(media>)
	return media;
}


