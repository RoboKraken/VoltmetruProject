#include <stdlib.h>
#include "lcd_st7565.h"

//Buffer with pre-filled Adafruit-Logo
uint8_t buffer[] = {
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x3, 0x7, 0xF, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x7, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x7F, 0x3F, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x1F, 0x3F, 0x70, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x6, 0x0, 0x0, 0x0, 0x3, 0x3,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xF, 0x7, 0x7,
0x7, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3E, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0x3F,
0x70, 0x60, 0x60, 0x60, 0x60, 0x30, 0x7F, 0x3F, 0x0, 0x0, 0x1F, 0x3F, 0x70, 0x60, 0x60, 0x60,
0x60, 0x39, 0xFF, 0xFF, 0x0, 0x6, 0x1F, 0x39, 0x60, 0x60, 0x60, 0x60, 0x30, 0x3F, 0x7F, 0x0,
0x0, 0x60, 0xFF, 0xFF, 0x60, 0x60, 0x0, 0x7F, 0x7F, 0x70, 0x60, 0x60, 0x40, 0x0, 0x7F, 0x7F,
0x0, 0x0, 0x0, 0x0, 0x7F, 0x7F, 0x0, 0x0, 0x0, 0x7F, 0x7F, 0x0, 0x0, 0x60, 0xFF, 0xFF,
0x60, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x80, 0xF8, 0xFC, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xE7, 0xE7, 0xE3,
0xF3, 0xF9, 0xFF, 0xFF, 0xFF, 0xF7, 0x7, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF,
0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x3F, 0x3F, 0x1F, 0xF, 0x7, 0x3, 0x0, 0x0, 0x0, 0xC0,
0xE0, 0x60, 0x20, 0x20, 0x60, 0xE0, 0xE0, 0xE0, 0x0, 0x0, 0x80, 0xC0, 0xE0, 0x60, 0x20, 0x60,
0x60, 0xE0, 0xE0, 0xE0, 0x0, 0x0, 0x80, 0xC0, 0x60, 0x60, 0x20, 0x60, 0x60, 0xE0, 0xE0, 0x0,
0x0, 0x0, 0xE0, 0xE0, 0x0, 0x0, 0x0, 0xE0, 0xE0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xE0,
0x60, 0x60, 0x60, 0x60, 0xE0, 0x80, 0x0, 0x0, 0x0, 0xE0, 0xE0, 0x0, 0x0, 0x0, 0xE0, 0xE0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x3, 0x7, 0x1F, 0x9F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xF1, 0xE3,
0xE3, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xFC, 0x7F, 0x3F, 0x3F, 0x3F, 0x3F, 0x7F, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF0, 0xE0, 0x80, 0x0, 0x0, 0x0, 0xC,
0x1C, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7F, 0x7F, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7, 0x7, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1C, 0xC, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFE, 0xFC, 0xF8,
0xF8, 0xF0, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF,
0xFF, 0x0, 0x0, 0x0, 0xFF, 0xFF, 0xE0, 0xC0, 0xC0, 0xC0, 0xFF, 0x7F, 0x0, 0x0, 0x1E, 0x7F,
0xE1, 0xC0, 0xC0, 0xC0, 0xC0, 0x61, 0xFF, 0xFF, 0x0, 0x0, 0xFE, 0xFF, 0x1, 0x0, 0x0, 0x0,
0xFF, 0xFF, 0x0, 0x0, 0x21, 0xF9, 0xF8, 0xDC, 0xCC, 0xCF, 0x7, 0x0, 0xC0, 0xFF, 0xFF, 0xC0,
0x80, 0x0, 0xFF, 0xFF, 0xC0, 0xC0, 0x80, 0x0, 0x0, 0xFF, 0xFF, 0x0, 0x0, 0x1F, 0x7F, 0xF9,
0xC8, 0xC8, 0xC8, 0xC8, 0x79, 0x39, 0x0, 0x0, 0x71, 0xF9, 0xD8, 0xCC, 0xCE, 0x47, 0x3, 0x0,
0x0, 0x0, 0x0, 0x0, 0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xF8, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF0, 0xC0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xC0,
0xC0, 0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0x80,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0x80, 0xC0, 0xC0, 0xC0, 0xC0,
0xC0, 0x80, 0x0, 0x0, 0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0,
0x0, 0x0, 0xC0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0x80, 0xC0,
0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x0, 0x0, 0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
};

uint8_t pagemap[] = { 7, 6, 5, 4, 3, 2, 1, 0 }; // checked the order on Arduino UNO

extern SPI_HandleTypeDef hspi1;

#define LCDWIDTH 128
#define LCDHEIGHT 64
#define CMD(c)        do { HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 0 ); st7565_sendbyte( c ); } while (0);
#define DATA(d)       do { HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 1 ); st7565_sendbyte( d ); } while (0);
#define DELAY(mS)     do { HAL_Delay( mS ); } while(0);

//Renders the buffer contents
#if ST7565_ASINCRON_TRANSMIT == ST7565_TRUE
void st7565_write_buffer(uint8_t *buffer)
{
    static PageType page = Page0;
    static uint8_t Data[16*64] = {0};
    uint8_t DataToSend[128] = {0};
    if(hspi1.State == HAL_SPI_STATE_READY)
    {
        switch(page)
        {
            case Page0:
            {
                for(uint16_t i = 0; i< sizeof(Data); i++)    //update all the display content
                {
                    Data[i] = buffer[i];
                }
                CMD(ST7565_CMD_SET_PAGE | pagemap[page]);
                CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
                CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
                CMD(ST7565_CMD_RMW);
                HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 1 );
                //HAL_SPI_Transmit(&hspi1, (uint8_t *)0xff, 1, 100);
                for(uint16_t i = 0; i< 128; i++)    //update page
                {
                    DataToSend[i]=Data[(128 * page) + i];
                }
                HAL_SPI_Transmit(&hspi1, (uint8_t *)DataToSend, 128, 100);
                page = Page1;
                break;
            }
            case Page1:
            {
                CMD(ST7565_CMD_SET_PAGE | pagemap[page]);
                CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
                CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
                CMD(ST7565_CMD_RMW);
                HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 1 );
                //HAL_SPI_Transmit(&hspi1, (uint8_t *)0xff, 1, 100);
                for(uint8_t i = 0; i< 128; i++)    //update page
                {
                    DataToSend[i]=Data[(128 * page) + i];
                }
                HAL_SPI_Transmit(&hspi1, (uint8_t *)DataToSend, 128, 100);
                page = Page2;
                break;
            }
            case Page2:
            {
                CMD(ST7565_CMD_SET_PAGE | pagemap[page]);
                CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
                CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
                CMD(ST7565_CMD_RMW);
                HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 1 );
                //HAL_SPI_Transmit(&hspi1, (uint8_t *)0xff, 1, 100);
                for(uint8_t i = 0; i< 128; i++)    //update page
                {
                    DataToSend[i]=Data[(128 * page) + i];
                }
                HAL_SPI_Transmit(&hspi1, (uint8_t *)DataToSend, 128, 100);
                page = Page3;
                break;
            }
            case Page3:
            {
                CMD(ST7565_CMD_SET_PAGE | pagemap[page]);
                CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
                CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
                CMD(ST7565_CMD_RMW);
                HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 1 );
                //HAL_SPI_Transmit(&hspi1, (uint8_t *)0xff, 1, 100);
                for(uint8_t i = 0; i< 128; i++)    //update page
                {
                    DataToSend[i]=Data[(128 * page) + i];
                }
                HAL_SPI_Transmit(&hspi1, (uint8_t *)DataToSend, 128, 100);
                page = Page4;
                break;
            }
            case Page4:
            {
                CMD(ST7565_CMD_SET_PAGE | pagemap[page]);
                CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
                CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
                CMD(ST7565_CMD_RMW);
                HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 1 );
               // HAL_SPI_Transmit(&hspi1, (uint8_t *)0xff, 1, 100);
                for(uint8_t i = 0; i< 128; i++)    //update page
                {
                    DataToSend[i]=Data[(128 * page) + i];
                }
                HAL_SPI_Transmit(&hspi1, (uint8_t *)DataToSend, 128, 100);
                page = Page5;
                break;
            }
            case Page5:
            {
                CMD(ST7565_CMD_SET_PAGE | pagemap[page]);
                CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
                CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
                CMD(ST7565_CMD_RMW);
                HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 1 );
                //HAL_SPI_Transmit(&hspi1, (uint8_t *)0xff, 1, 100);
                for(uint8_t i = 0; i< 128; i++)    //update page
                {
                    DataToSend[i]=Data[(128 * page) + i];
                }
                HAL_SPI_Transmit(&hspi1, (uint8_t *)DataToSend, 128, 100);
                page = Page6;
                break;
            }
            case Page6:
            {
                CMD(ST7565_CMD_SET_PAGE | pagemap[page]);
                CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
                CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
                CMD(ST7565_CMD_RMW);
                HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 1 );
                //HAL_SPI_Transmit(&hspi1, (uint8_t *)0xff, 1, 100);
                for(uint8_t i = 0; i< 128; i++)    //update page
                {
                    DataToSend[i]=Data[(128 * page) + i];
                }
                HAL_SPI_Transmit(&hspi1, (uint8_t *)DataToSend, 128, 100);
                page = Page7;
                break;
            }
            case Page7:
            {
                CMD(ST7565_CMD_SET_PAGE | pagemap[page]);
                CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
                CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
                CMD(ST7565_CMD_RMW);
                HAL_GPIO_WritePin( SPICD_GPIO_Port, ST7565_A0_PIN, 1 );
                //HAL_SPI_Transmit(&hspi1, (uint8_t *)0xff, 1, 100);
                for(uint8_t i = 0; i< 128; i++)    //update page
                {
                    DataToSend[i]=Data[(128 * page) + i];
                }
                HAL_SPI_Transmit(&hspi1, (uint8_t *)DataToSend, 128, 100);
                page = Page0;
                break;
            }
        }
    }
}
#else
void st7565_write_buffer(uint8_t *buffer)
{
    uint8_t p;
    for (p = 0; p < 8; p++) {
        CMD(ST7565_CMD_SET_PAGE | pagemap[p]);
        CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
        CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
        CMD(ST7565_CMD_RMW);
        HAL_GPIO_WritePin(SPICD_GPIO_Port, ST7565_A0_PIN, 1);
        HAL_SPI_Transmit(&hspi1, &buffer[128 * p], 128, 6);
    }
}
#endif

// Simulates an SPI write using GPIO
// with byte, The byte to send
void st7565_sendbyte(uint8_t byte) {

	HAL_SPI_Transmit(&hspi1, &byte, 1, 100);
}

//Initialises the ST7565 LCD display
void st7565_init(void) {
	// Set clock pin to high
//	HAL_GPIO_WritePin(SPISCLK_GPIO_Port, ST7565_SCLK_PIN, 1);
	// Set data pin to output and high
//	HAL_GPIO_WritePin(SPISDAT_GPIO_Port, ST7565_SDAT_PIN, 1);
	// Configure backlight pin to output and set high (off)
	HAL_GPIO_WritePin(BL_GPIO_Port, ST7565_BL_PIN, 1);

	// Configure A0 pin to output and set high
	HAL_GPIO_WritePin(SPICD_GPIO_Port, ST7565_A0_PIN, 1);

	// Configure Reset pin and set high
	HAL_GPIO_WritePin(SPIRST_GPIO_Port, ST7565_RST_PIN, 1);

	// Configure select pin and set high
	HAL_GPIO_WritePin(SPICS_GPIO_Port, ST7565_CS_PIN, 1);

	// Reset
	HAL_GPIO_WritePin(SPICS_GPIO_Port, ST7565_CS_PIN, 0);    // Set CS low
	HAL_GPIO_WritePin(SPIRST_GPIO_Port, ST7565_RST_PIN, 0);  // Set reset low
	DELAY(500);                                              // Wait 500ms
	HAL_GPIO_WritePin(SPIRST_GPIO_Port, ST7565_RST_PIN, 1);  // Set reset high

	// Configure Display
	CMD(ST7565_CMD_SET_BIAS_7);                      // LCD Bias Select
	CMD(ST7565_CMD_SET_ADC_NORMAL);                  // ADC Select
	CMD(ST7565_CMD_SET_COM_NORMAL);                  // SHL Select
	CMD(ST7565_CMD_SET_DISP_START_LINE);             // Initial Display Line
	CMD(ST7565_CMD_SET_POWER_CONTROL | 0x04);        // Turn on voltage converter (VC=1, VR=0, VF=0)
	DELAY(50);                						 // Wait 50ms
	CMD(ST7565_CMD_SET_POWER_CONTROL | 0x06);        // Turn on voltage regulator (VC=1, VR=1, VF=0)
	DELAY(50);                                       // Wait 50ms
	CMD(ST7565_CMD_SET_POWER_CONTROL | 0x07);        // Turn on voltage follower
	DELAY(10);                                       // Wait 10ms
	CMD(ST7565_CMD_SET_RESISTOR_RATIO | 0x6);        // Set LCD operating voltage

	// Turn display on
	CMD(ST7565_CMD_DISPLAY_ON);
	CMD(ST7565_CMD_SET_ALLPTS_NORMAL);
	st7565_set_brightness(0x00);   // arduino 0x18
 	st7565_clear_screen();

	//Write the Adafruit-Logo, clear the buffer, wait and fade out
	//st7565_write_buffer(buffer);
	st7565_clear_buffer(buffer);
	//DELAY(800);
	//st7565_fade_out(24);
}

//Enables the backlight
void st7565_backlight_enable(void) {
	HAL_GPIO_WritePin( BL_GPIO_Port, ST7565_BL_PIN, 1);
}

///Disables the backlight
void st7565_backlight_disable(void) {
	HAL_GPIO_WritePin( BL_GPIO_Port, ST7565_BL_PIN, 0);
}

//Sets the display brightness
void st7565_set_brightness(uint8_t val) {
	CMD(ST7565_CMD_SET_VOLUME_FIRST);
	CMD(ST7565_CMD_SET_VOLUME_SECOND | (val & 0x3f));
}

// Fade out from max 64 to 0
void st7565_fade_out(uint8_t level) {
	uint8_t i = 0;
	for (i = level; i > 0; i--) {
		st7565_set_brightness(i);
		DELAY(100);
	}
}

// Fade in from 0 to max 64
void st7565_fade_in(uint8_t level) {
	uint8_t i = 0;
	for (i = 0; i < level; i++) {
		st7565_set_brightness(i);
		DELAY(100);
	}
}

//Clears the screen
void st7565_clear_screen(void) {
	uint8_t p, c;
	for (p = 0; p < 8; p++) {
		CMD(ST7565_CMD_SET_PAGE | p);
		for (c = 0; c < 129; c++) {
			CMD(ST7565_CMD_SET_COLUMN_LOWER | (c & 0xf));
			CMD(ST7565_CMD_SET_COLUMN_UPPER | ((c >> 4) & 0xf));
			DATA(0x0);
		}
	}
}

// clear everything
void st7565_clear_buffer(uint8_t *buff) {
	memset(buff, 0, 1024);
}

#define ANZAHL 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
void st7565_drawfallingbitmap(uint8_t *buff, const uint8_t *bitmap, uint8_t w,
		uint8_t h) {
	uint8_t icons[ANZAHL][3];
	srand(buff[666]);
// initialize
	uint8_t f = 0;
	for (f = 0; f < ANZAHL; f++) {
		icons[f][XPOS] = rand() % 128;
		icons[f][YPOS] = 0;
		icons[f][DELTAY] = rand() % 5 + 1;
	}
	while (1) {
// draw each icon
		for (f = 0; f < ANZAHL; f++) {
			st7565_drawbitmap(buffer, icons[f][XPOS], icons[f][YPOS], bitmap, w,
					h, 1);
		}
		DELAY(200);
		st7565_write_buffer(buffer);
// then erase it + move it
		for (f = 0; f < ANZAHL; f++) {
			st7565_drawbitmap(buffer, icons[f][XPOS], icons[f][YPOS], bitmap, w,
					h, 0);
// move it
			icons[f][YPOS] += icons[f][DELTAY];
// if its gone, reinit
			if (icons[f][YPOS] > 64) {
				icons[f][XPOS] = rand() % 128;
				icons[f][YPOS] = 0;
				icons[f][DELTAY] = rand() % 5 + 1;
			}
		}
	}
}

//set *pixel on *x/*y with *color
void st7565_setpixel(uint8_t *buff, uint8_t x, uint8_t y, uint8_t color) {
	if ((x >= LCDWIDTH) || (y >= LCDHEIGHT))
		return;
// x is which column

	if (color)
		buff[x + (y / 8) * 128] |= (1 << (7 - (y % 8)));
	else
		buff[x + (y / 8) * 128] &= ~(1 << (7 - (y % 8)));
}

//draw pixel at *x/*y from *bitmap
void st7565_drawbitmap(uint8_t *buff, uint8_t x, uint8_t y,
		const uint8_t *bitmap, uint8_t w, uint8_t h, uint8_t color) {
	uint8_t j, i;
	for (j = 0; j < h; j++) {
		for (i = 0; i < w; i++) {
			if (*(bitmap + i + (j / 8) * w) & (1 << (j % 8))) {
				st7565_setpixel(buff, x + i, y + j, color);
			}
		}
	}
}

//draw a sting at *x/*y
void st7565_drawstring(uint8_t *buff, uint8_t x, uint8_t line, uint8_t *c,uint8_t fontSelect) {
	while (c[0] != 0) {
		st7565_drawchar(buff, x, line, c[0], fontSelect);
		c++;
		x += 6; // 6 pixels wide
		if (x + 6 >= LCDWIDTH) {
			x = 0; // ran out of this line
			line++;
		}
		if (line >= (LCDHEIGHT / 8))
			return; // ran out of space :(
	}
}

//draw a single character at *x/*y
void st7565_drawchar(uint8_t *buff, uint8_t x, uint8_t line, uint8_t c, uint8_t fontSelect) {
	uint8_t i;
	for (i = 0; i < 5; i++) {
		if(fontSelect==0)
		buff[x + (line * 128)] = *(lcdfont5x7 + (c * 5) + i);
		else if(fontSelect==1)
		buff[x + (line * 128)] = *(ultraFont + (c * 5) + i);
		x++;
	}
}

// the most basic function, clear a single pixel
void st7565_clearpixel(uint8_t *buff, uint8_t x, uint8_t y) {
// x is which column
	buff[x + (y / 8) * 128] &= ~(1 << (7 - (y % 8)));
}

// Draw a line, based on bresenham's algorithm
void st7565_drawline_complex(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t x1,
		uint8_t y1, uint8_t color) {
	uint8_t tmp, swap = 0;
	uint8_t x, y;
	uint8_t dx, dy;
	int8_t ystep, err;

	if (x0 > x1)
		dx = x0 - x1;
	else
		dx = x1 - x0;
	if (y0 > y1)
		dy = y0 - y1;
	else
		dy = y1 - y0;

	if (dy > dx) {
		swap = 1;
		tmp = dx;
		dx = dy;
		dy = tmp;
		tmp = x0;
		x0 = y0;
		y0 = tmp;
		tmp = x1;
		x1 = y1;
		y1 = tmp;
	}
	if (x0 > x1) {
		tmp = x0;
		x0 = x1;
		x1 = tmp;
		tmp = y0;
		y0 = y1;
		y1 = tmp;
	}

	err = dx >> 1;
	if (y1 > y0)
		ystep = 1;
	else
		ystep = -1;
	y = y0;
	for (x = x0; x <= x1; x++) {
		if (swap == 0)
			st7565_setpixel(buff, x, y, color);
		else
			st7565_setpixel(buff, y, x, color);
		err -= (uint8_t) dy;
		if (err < 0) {
			y += (uint8_t) ystep;
			err += (uint8_t) dx;
		}
	}
}

// draw a filled rectangle
void st7565_fillrect(uint8_t *buff, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
    if (x >= LCDWIDTH || y >= LCDHEIGHT)
        return;
    if (x + w > LCDWIDTH)
        w = LCDWIDTH - x;
    if (y + h > LCDHEIGHT)
        h = LCDHEIGHT - y;
    uint8_t y_start = y;
    uint8_t y_end = y + h - 1;
    uint8_t page_start = y_start / 8;
    uint8_t page_end = y_end / 8;
    uint8_t bit_start = y_start % 8;
    uint8_t bit_end = y_end % 8;
    for (uint8_t px = x; px < x + w; px++) {
        for (uint8_t page = page_start; page <= page_end; page++) {
            uint8_t mask = 0xFF;
            if (page == page_start) {
                mask &= (0xFF >> bit_start);
            }
            if (page == page_end) {
                mask &= (0xFF << (7 - bit_end));
            }
            uint16_t idx = px + page * 128;
            if (color)
                buff[idx] |= mask;
            else
                buff[idx] &= ~mask;
        }
    }
}

// draw a rectangle
void st7565_drawrect(uint8_t *buff, uint8_t x, uint8_t y, uint8_t w, uint8_t h,
		uint8_t color) {
// stupidest version - just pixels - but fast with internal buffer!
	uint8_t i;
	for (i = x; i < x + w; i++) {
		st7565_setpixel(buff, i, y, color);
		st7565_setpixel(buff, i, y + h - 1, color);
	}
	for (i = y; i < y + h; i++) {
		st7565_setpixel(buff, x, i, color);
		st7565_setpixel(buff, x + w - 1, i, color);
	}
}

// draw a circle
void st7565_drawcircle(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t r,
		uint8_t color) {
	int8_t f = 1 - r;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * r;
	int8_t x = 0;
	int8_t y = r;
	st7565_setpixel(buff, x0, y0 + r, color);
	st7565_setpixel(buff, x0, y0 - r, color);
	st7565_setpixel(buff, x0 + r, y0, color);
	st7565_setpixel(buff, x0 - r, y0, color);
	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		st7565_setpixel(buff, x0 + x, y0 + y, color);
		st7565_setpixel(buff, x0 - x, y0 + y, color);
		st7565_setpixel(buff, x0 + x, y0 - y, color);
		st7565_setpixel(buff, x0 - x, y0 - y, color);
		st7565_setpixel(buff, x0 + y, y0 + x, color);
		st7565_setpixel(buff, x0 - y, y0 + x, color);
		st7565_setpixel(buff, x0 + y, y0 - x, color);
		st7565_setpixel(buff, x0 - y, y0 - x, color);
	}
}

// draw a filled circle
void st7565_fillcircle(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t r,
		uint8_t color) {
	int8_t f = 1 - r;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * r;
	int8_t x = 0;
	int8_t y = r;
	uint8_t i;
	for (i = y0 - r; i <= y0 + r; i++) {
		st7565_setpixel(buff, x0, i, color);
	}
	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;
		for (i = y0 - y; i <= y0 + y; i++) {
			st7565_setpixel(buff, x0 + x, i, color);
			st7565_setpixel(buff, x0 - x, i, color);
		}
		for (i = y0 - x; i <= y0 + x; i++) {
			st7565_setpixel(buff, x0 + y, i, color);
			st7565_setpixel(buff, x0 - y, i, color);
		}
	}
}

void st7565_drawline(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color) {
    if (x0 == x1) {
        if (x0 >= LCDWIDTH)
            return;
        uint8_t y_start = y0 < y1 ? y0 : y1;
        uint8_t y_end = y0 > y1 ? y0 : y1;
        if (y_end >= LCDHEIGHT)
            y_end = LCDHEIGHT - 1;
        uint8_t page_start = y_start / 8;
        uint8_t page_end = y_end / 8;
        uint8_t bit_start = y_start % 8;
        uint8_t bit_end = y_end % 8;
        for (uint8_t page = page_start; page <= page_end; page++) {
            uint8_t mask = 0xFF;
            if (page == page_start) {
                mask &= (0xFF >> bit_start);
            }
            if (page == page_end) {
                mask &= (0xFF << (7 - bit_end));
            }
            uint16_t idx = x0 + page * 128;
            if (color)
                buff[idx] |= mask;
            else
                buff[idx] &= ~mask;
        }
    } else if (y0 == y1) {
        if (y0 >= LCDHEIGHT)
            return;
        uint8_t x_start = x0 < x1 ? x0 : x1;
        uint8_t x_end = x0 > x1 ? x0 : x1;
        if (x_end >= LCDWIDTH)
            x_end = LCDWIDTH - 1;
        uint8_t page = y0 / 8;
        uint8_t bit = 7 - (y0 % 8);
        uint8_t mask = 1 << bit;
        uint16_t idx = x_start + page * 128;
        // 32-bit write optimization
        uint32_t *p32 = (uint32_t *)&buff[idx];
        uint32_t val = color ? 0xFFFFFFFF : 0x00000000;
        uint8_t rem = (x_end - x_start + 1);
        while (rem >= 4 && (((uintptr_t)p32 & 0x3) == 0)) {
            if (color) *p32 |= (val & (mask * 0x01010101));
            else *p32 &= ~(mask * 0x01010101);
            p32++;
            rem -= 4;
        }
        idx = (uint8_t *)p32 - &buff[x_start + page * 128];
        for (uint8_t i = 0; i < rem; i++) {
            if (color) buff[x_start + page * 128 + idx + i] |= mask;
            else buff[x_start + page * 128 + idx + i] &= ~mask;
        }
    }
}
