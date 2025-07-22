/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <VoltInterpolation.c>
#include <MAFilterButton.c>
#include <MAFilterVolt.c>
#include "Rte.h"
#include "lcd_st7565.h"
#include "lcd_st7565_pinconf.h"
#include "font.h"
#include <stdio.h>
#include <string.h>
#include "simple_os.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_BUFFER_SIZE_BUTTON 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/*ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;*/

//UART_HandleTypeDef huart2;

/* Definitions for readAdcVolt */
/*osThreadId_t readAdcVoltHandle;
const osThreadAttr_t readAdcVolt_attributes = {
  .name = "readAdcVolt",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};*/
/* Definitions for displayVoltRead */
/*osThreadId_t displayVoltReadHandle;
const osThreadAttr_t displayVoltRead_attributes = {
  .name = "displayVoltRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};*/
/* Definitions for readButton */
/*osThreadId_t readButtonHandle;
const osThreadAttr_t readButton_attributes = {
  .name = "readButton",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/* USER CODE BEGIN PFP */
void readAdcVoltFunction(void);
void displayVoltReadFunction(void);
void readButtonFunction(void);
void init_task(void);
void test_task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t test = 0;
uint8_t flag=0;
uint8_t buttonTaskCreated = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);
  /* USER CODE END 2 */

  uint8_t nrTasks=3; //Numar taskuri
  SimpleTask tasks[] = {
    {"readAdcVoltFunction", readAdcVoltFunction, 100},
	{"displayVoltReadFunction", displayVoltReadFunction, 1570},
    {"readButtonFunction",readButtonFunction,100}
  };//timpul total pana vom intra din nou intr-o functie, ex readAdcVoltFunction, e suma tuturor us a tuturor taskurilor.

  uint32_t initTaskMaxTime=1570*1000; //timp alocat task-ului de init OS(dupa initializarea OS-ului in sine). In us.
  OS_Init(tasks, nrTasks, init_task,initTaskMaxTime);
  OS_Run();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Facem OS custom aici, avem un init task dupa care vin celalalte.
void init_task(void) {
    test = 42;
    st7565_init();
      			  st7565_backlight_enable();
      			  st7565_clear_screen();
      			st7565_set_brightness(0);
      			  //st7565_fade_out(64);
      			//st7565_write_buffer(buffer); // This will show the pre-filled logo
      			//osDelay(1000);

      			//st7565_fade_in(10);


      			  st7565_clear_buffer(buffer);

      			    			  //Animatie jmekera de startup

      			  uint8_t spacingx=126/10; //Cat de distantate sunt liniile in animatie x
      			  uint8_t spacingy=63/6;
      			  for(uint16_t i=0;i<=63;i+=2){//i,j stanga sus->centru
      				  //spacing=5+i/10;
      				  if(i%4==0)st7565_set_brightness(i/4);
      				  uint16_t j=i/2;
      				  st7565_clear_buffer(buffer);
      				  //Linie de la i,j la marginea dreapta
      				  for(uint16_t j2=0;j2<=63;j2+=spacingy){
      					st7565_drawline_complex(buffer,i,j,126,j2,1);
      				  }
      				//Linie de la i,j la marginea stanga
      				  				  for(uint16_t j2=0;j2<=63;j2+=spacingy){
      				  					st7565_drawline_complex(buffer,i,j,0,j2,1);
      				  				  }

      				  //Linie de la i,j la margine jos
      				  for(uint16_t i2=0;i2<=126;i2+=spacingx){
      				  		st7565_drawline_complex(buffer,i,j,i2,63,1);
      				  				  }
      				//Linie de la i,j la margine sus
      				  				  for(uint16_t i2=0;i2<=126;i2+=spacingx){
      				  				  		st7565_drawline_complex(buffer,i,j,i2,0,1);
      				  				  }

      				//deseneaza frame
      				st7565_write_buffer(buffer);
    				//HAL_Delay(50);

      				//osDelay(2);

      			  }
      			for(uint16_t i=64;i<=126;i+=2){//i,j centru->dreapta jos
      				//spacing=11-(i-64)/10;
      				if(i%4==0)st7565_set_brightness((126-i)/4);
      			  				  uint16_t j=i/2;
      			  				  st7565_clear_buffer(buffer);
      			  				//Linie de la i,j la marginea dreapta
      			  				  				  for(uint16_t j2=0;j2<=63;j2+=spacingy){
      			  				  					st7565_drawline_complex(buffer,i,j,126,j2,1);
      			  				  				  }
      			  				  				//Linie de la i,j la marginea stanga
      			  				  				  				  for(uint16_t j2=0;j2<=63;j2+=spacingy){
      			  				  				  					st7565_drawline_complex(buffer,i,j,0,j2,1);
      			  				  				  				  }

      			  				  				  //Linie de la i,j la margine jos
      			  				  				  for(uint16_t i2=0;i2<=126;i2+=spacingx){
      			  				  				  		st7565_drawline_complex(buffer,i,j,i2,63,1);
      			  				  				  }
      			  				  				//Linie de la i,j la margine sus
      			  				  				  				  for(uint16_t i2=0;i2<=126;i2+=spacingx){
      			  				  				  				  		st7565_drawline_complex(buffer,i,j,i2,0,1);
      			  				  				  				  }

      			  				//deseneaza frame
      			  				st7565_write_buffer(buffer);
    			  				//HAL_Delay(50);

      			  				//osDelay(2);

      			  			  }
      			//st7565_fade_in(64);
      			  //st7565_drawstring(uint8_t *buff, uint8_t x, uint8_t line, uint8_t *c);
      			  //st7565_drawstring(buffer,15,2,"Hello World!!");


      			  //HAL_ReadPin
      			  //Trimitem comanda sa desenam

      			st7565_set_brightness(0);


    
}
/* USER CODE END 4 */




/* USER CODE BEGIN Header_readAdcVoltFunction */
/**
  * @brief  Function implementing the readAdcVolt thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_readAdcVoltFunction */
void readAdcVoltFunction(void)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */

	      /*hadc.Instance->CHSELR = 1<<ADC_CHANNEL_1;
	      HAL_ADC_Start(&hadc);
	      if(HAL_ADC_PollForConversion(&hadc, 4)==HAL_OK)
	      voltReadRaw = HAL_ADC_GetValue(&hadc);
	      else voltReadRaw=0;

	      HAL_ADC_Stop(&hadc);*/

			voltReadRaw=adc_buffer[255];
	      voltRead = filterVolt(interpolation(voltReadRaw));

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_displayVoltReadFunction */
/**
* @brief Function implementing the displayVoltRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayVoltReadFunction */
void displayVoltReadFunction(void)
{
    enum { DRAWING, DRAWING_POPUP, SENDING_PAGE, WAITING };
    static uint8_t state = DRAWING;
    static uint8_t current_page = 0;
    static uint32_t last_frame_time = 0;
    uint32_t now = HAL_GetTick_us();
    switch(state) {
        case DRAWING:
            st7565_clear_buffer(buffer);
            if(displayMode==0){
                st7565_drawstring(buffer,0,0,"Volt:",fontMode);
                char volt[100];
                itoa(voltRead,volt,10);
                if(voltRead<10){
                    volt[4]='\0';
                    volt[3]=volt[0];
                    volt[2]='0';
                    volt[1]='.';
                    volt[0]='0';
                }
                else if(voltRead>=10&&voltRead<100){
                    volt[4]='\0';
                    volt[3]=volt[1];
                    volt[2]=volt[0];
                    volt[1]='.';
                    volt[0]='0';
                }
                else{
                    volt[4]='\0';
                    volt[3]=volt[2];
                    volt[2]=volt[1];
                    volt[1]='.';
                }
                st7565_drawstring(buffer,0,1,volt,fontMode);
            } else if(displayMode==1){
                st7565_drawstring(buffer,30,2,"Volt Range",fontMode);
                int bar_x0 = 5;
                int bar_x1 = 121;
                int bar_y0 = 30;
                int bar_y1 = 37;
                int squares = 10;
                int inner_x0 = bar_x0 + 1;
                int inner_x1 = bar_x1 - 1;
                int inner_y0 = bar_y0 + 1;
                int inner_y1 = bar_y1 - 1;
                int inner_width = inner_x1 - inner_x0 + 1;
                int square_width = inner_width / squares;
                int remainder = inner_width - square_width * squares;
                int volt_step = 330 / squares;
                int x = inner_x0;
                for(int i = 0; i < squares; i++) {
                    int w = square_width + (i < remainder ? 1 : 0);
                    int threshold = (i + 1) * volt_step;
                    if(voltRead >= threshold) {
                        st7565_fillrect(buffer, x, inner_y0, w, inner_y1 - inner_y0 + 1, 1);
                    }
                    x += w;
                }
                st7565_drawline(buffer, bar_x0+1, bar_y0, bar_x1-1, bar_y0, 1);
                st7565_drawline(buffer, bar_x0+1, bar_y1, bar_x1-1, bar_y1, 1);
                st7565_drawline(buffer, bar_x0, bar_y0+1, bar_x0, bar_y1-1, 1);
                st7565_drawline(buffer, bar_x1, bar_y0+1, bar_x1, bar_y1-1, 1);
                st7565_setpixel(buffer, bar_x0 +1, bar_y0+1, 1);
                st7565_setpixel(buffer, bar_x0 +1, bar_y1-1, 1);
                st7565_setpixel(buffer, bar_x1 -1, bar_y0+1, 1);
                st7565_setpixel(buffer, bar_x1 -1, bar_y1-1, 1);
                st7565_drawstring(buffer, 0, 5, "0",fontMode);
                st7565_drawstring(buffer, 20, 5, "0.8",fontMode);
                st7565_drawstring(buffer, 45, 5, "1.6",fontMode);
                st7565_drawstring(buffer, 75, 5, "2.5",fontMode);
                st7565_drawstring(buffer, 108, 5, "3.3",fontMode);
            } else if(displayMode==2){
                for(int x=1; x<256; x+=2) {
                    int value = adc_buffer[x];
                    int y = 63 - (value * 63 / 4095);
                    if(y<0) y=0;
                    if(y>63) y=63;
                    st7565_setpixel(buffer, (x-1)/2, y, 1);
                }
            } else if(displayMode==100){
                st7565_drawstring(buffer, 0, 0, " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~\x7F",fontMode);
            }
            state = DRAWING_POPUP;
            break;
        case DRAWING_POPUP:
            if(showDisplayModeOverlay && (now - displayModeChangeTime) < 1250*1000) {
                uint8_t rect_x = 80;
                uint8_t rect_y = 45;
                uint8_t rect_w = 47;
                uint8_t rect_h = 18;
                st7565_fillrect(buffer, rect_x, rect_y, rect_w, rect_h, 0);
                st7565_drawline(buffer, rect_x, rect_y, rect_x + rect_w, rect_y, 1);
                st7565_drawline(buffer, rect_x, rect_y, rect_x, rect_y + rect_h, 1);
                st7565_drawline(buffer, rect_x + rect_w, rect_y, rect_x + rect_w, rect_y + rect_h, 1);
                st7565_drawline(buffer, rect_x, rect_y + rect_h, rect_x + rect_w, rect_y + rect_h, 1);
                char mode_text[20] = "Mode  ";
                mode_text[5] = displayMode + '0';
                st7565_drawstring(buffer, rect_x + 2, rect_y/8 + 1, mode_text, fontMode);
            } else {
                showDisplayModeOverlay = 0;
            }
            state = SENDING_PAGE;
            current_page = 0;
            break;
        case SENDING_PAGE:
            CMD(ST7565_CMD_SET_PAGE | pagemap[current_page]);
            CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
            CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
            CMD(ST7565_CMD_RMW);
            HAL_GPIO_WritePin(SPICD_GPIO_Port, ST7565_A0_PIN, 1);
            HAL_SPI_Transmit(&hspi1, &buffer[128 * current_page], 128, 6);
            current_page++;
            /*CMD(ST7565_CMD_SET_PAGE | pagemap[current_page]);
            CMD(ST7565_CMD_SET_COLUMN_LOWER | (0x0 & 0xf));
            CMD(ST7565_CMD_SET_COLUMN_UPPER | ((0x0 >> 4) & 0xf));
            CMD(ST7565_CMD_RMW);
            HAL_GPIO_WritePin(SPICD_GPIO_Port, ST7565_A0_PIN, 1);
            HAL_SPI_Transmit(&hspi1, &buffer[128 * current_page], 128, 6);
            current_page++;*/
            if (current_page >= 8) {
            	os_debug_drawing_time=now - last_frame_time;
                state = WAITING;
            }
            break;
        case WAITING:
            if (now - last_frame_time >= 42*1000) {
                state = DRAWING;
                last_frame_time = now;
            }
            break;
    }
}

/* USER CODE BEGIN Header_readButtonFunction */
/**
* @brief Function implementing the readButton thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readButtonFunction */
void readButtonFunction(void)
{
  /* USER CODE BEGIN readButtonFunction */
  /* Infinite loop */

    /*hadc.Instance->CHSELR = 1<<ADC_CHANNEL_0;
    if (HAL_ADC_Start(&hadc) == HAL_OK) {
      if (HAL_ADC_PollForConversion(&hadc, 4) == HAL_OK) {
        buttonReadRaw = HAL_ADC_GetValue(&hadc);
      } else {
        buttonReadRaw = 666;
      }
    } else {
      buttonReadRaw = 666;
    }
    HAL_ADC_Stop(&hadc);*/
	buttonReadRaw=adc_buffer[254];
    buttonRead[0]=buttonRead[1];
    buttonRead[1]=filterButton(interpolation(buttonReadRaw));
    
    uint8_t rawButtonState = 0;
    if(buttonRead[1] <= 50) {
        rawButtonState = 1;
    } else if(buttonRead[1] > 290 && buttonRead[1] < 315) {
        rawButtonState = 2;
    } else if(buttonRead[1] > 186 && buttonRead[1] < 206) {
        rawButtonState = 3;
    } else if(buttonRead[1] > 86 && buttonRead[1] < 108) {
        rawButtonState = 4;
    }
    
    if(rawButtonState != buttonState) {
        if(!buttonTransitionFlag) {
            buttonTransitionFlag = 1;
            buttonDebounceTimer = 0;
        }
        buttonDebounceTimer += 5;
        
        if(buttonDebounceTimer >= 15) {
            buttonStatePrev = buttonState;
            buttonState = rawButtonState;
            buttonTransitionFlag = 0;
            buttonDebounceTimer = 0;
            
            if(!buttonTransitionFlag) {
                if(buttonState == 1 && buttonStatePrev == 0) {
                    if(displayMode==0)displayMode=displayModeMax;
                    else displayMode--;
                    showDisplayModeOverlay = 1;
                    displayModeChangeTime = HAL_GetTick_us();
                }
                else if(buttonState == 2 && buttonStatePrev == 0) {
                    if(displayMode==displayModeMax)displayMode=0;
                    else displayMode++;
                    showDisplayModeOverlay = 1;
                    displayModeChangeTime = HAL_GetTick_us();
                }
                else if(buttonState == 3 && buttonStatePrev == 0) {//apas jos
                	if(fontMode==0)fontMode=fontModeMax;
                	else fontMode--;
                }
                else if(buttonState == 4 && buttonStatePrev == 0) {//apas sus
                	if(fontMode==fontModeMax)fontMode=0;
                	else fontMode++;
                }
            }
        }
    } else {
        buttonTransitionFlag = 0;
        buttonDebounceTimer = 0;
    }
  /* USER CODE END readButtonFunction */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
