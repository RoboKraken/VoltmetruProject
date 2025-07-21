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
#include "cmsis_os.h"

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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for readAdcVolt */
osThreadId_t readAdcVoltHandle;
const osThreadAttr_t readAdcVolt_attributes = {
  .name = "readAdcVolt",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for displayVoltRead */
osThreadId_t displayVoltReadHandle;
const osThreadAttr_t displayVoltRead_attributes = {
  .name = "displayVoltRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for readButton */
osThreadId_t readButtonHandle;
const osThreadAttr_t readButton_attributes = {
  .name = "readButton",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
void readAdcVoltFunction(void *argument);
void displayVoltReadFunction(void *argument);
void readButtonFunction(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */

  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of readAdcVolt */
  readAdcVoltHandle = osThreadNew(readAdcVoltFunction, NULL, &readAdcVolt_attributes);

  /* creation of displayVoltRead */
  displayVoltReadHandle = osThreadNew(displayVoltReadFunction, NULL, &displayVoltRead_attributes);

  /* creation of readButton */
  readButtonHandle = osThreadNew(readButtonFunction, NULL, &readButton_attributes);
  
  if (readButtonHandle != NULL) {
      buttonTaskCreated = 1;
  } else {
      buttonTaskCreated = 0;
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

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

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;

  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPICD_GPIO_Port, SPICD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BL_Pin|SPIRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPICS_GPIO_Port, SPICS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPICD_Pin */
  GPIO_InitStruct.Pin = SPICD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPICD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BL_Pin SPIRST_Pin */
  GPIO_InitStruct.Pin = BL_Pin|SPIRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPICS_Pin */
  GPIO_InitStruct.Pin = SPICS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPICS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_readAdcVoltFunction */
/**
  * @brief  Function implementing the readAdcVolt thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_readAdcVoltFunction */
void readAdcVoltFunction(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {


	      hadc.Instance->CHSELR = 1<<ADC_CHANNEL_1;
	      HAL_ADC_Start(&hadc);
	      if(HAL_ADC_PollForConversion(&hadc, 30)==HAL_OK)
	      voltReadRaw = HAL_ADC_GetValue(&hadc);
	      else voltReadRaw=0;

	      HAL_ADC_Stop(&hadc);

	      voltRead = filterVolt(interpolation(voltReadRaw));
    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_displayVoltReadFunction */
/**
* @brief Function implementing the displayVoltRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayVoltReadFunction */
void displayVoltReadFunction(void *argument)
{
  /* USER CODE BEGIN displayVoltReadFunction */
  /* Infinite loop */
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
  					st7565_drawline(buffer,i,j,126,j2,1);
  				  }
  				//Linie de la i,j la marginea stanga
  				  				  for(uint16_t j2=0;j2<=63;j2+=spacingy){
  				  					st7565_drawline(buffer,i,j,0,j2,1);
  				  				  }

  				  //Linie de la i,j la margine jos
  				  for(uint16_t i2=0;i2<=126;i2+=spacingx){
  				  		st7565_drawline(buffer,i,j,i2,63,1);
  				  				  }
  				//Linie de la i,j la margine sus
  				  				  for(uint16_t i2=0;i2<=126;i2+=spacingx){
  				  				  		st7565_drawline(buffer,i,j,i2,0,1);
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
  			  				  					st7565_drawline(buffer,i,j,126,j2,1);
  			  				  				  }
  			  				  				//Linie de la i,j la marginea stanga
  			  				  				  				  for(uint16_t j2=0;j2<=63;j2+=spacingy){
  			  				  				  					st7565_drawline(buffer,i,j,0,j2,1);
  			  				  				  				  }

  			  				  				  //Linie de la i,j la margine jos
  			  				  				  for(uint16_t i2=0;i2<=126;i2+=spacingx){
  			  				  				  		st7565_drawline(buffer,i,j,i2,63,1);
  			  				  				  }
  			  				  				//Linie de la i,j la margine sus
  			  				  				  				  for(uint16_t i2=0;i2<=126;i2+=spacingx){
  			  				  				  				  		st7565_drawline(buffer,i,j,i2,0,1);
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
  for(;;)
  {
	  //st7565_fillrect(buffer,10,10,10,10,1);
	  if(displayMode==0){
	  st7565_clear_buffer(buffer);
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
	    				//volt[0]=volt[0];
	    			}
	    			st7565_drawstring(buffer,0,1,volt,fontMode);

	  }

	  else if(displayMode==1){
      st7565_clear_buffer(buffer);
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
      for(int px = bar_x0 + 1; px < bar_x1; px++) {
          if(px != bar_x0 + 1 && px != bar_x1 - 1) {
              st7565_setpixel(buffer, px, bar_y0, 1);
              st7565_setpixel(buffer, px, bar_y1, 1);
          }
      }
      for(int py = bar_y0 + 1; py < bar_y1; py++) {
          if(py != bar_y0 + 1 && py != bar_y1 - 1) {
              st7565_setpixel(buffer, bar_x0, py, 1);
              st7565_setpixel(buffer, bar_x1, py, 1);
          }
      }
      st7565_setpixel(buffer, bar_x0 +1, bar_y0+1, 1);
      st7565_setpixel(buffer, bar_x0 +1, bar_y1-1, 1);

      st7565_setpixel(buffer, bar_x1 -1, bar_y0+1, 1);
      st7565_setpixel(buffer, bar_x1 -1, bar_y1-1, 1);
      st7565_drawstring(buffer, 0, 5, "0",fontMode);
      st7565_drawstring(buffer, 20, 5, "0.8",fontMode);
      st7565_drawstring(buffer, 45, 5, "1.6",fontMode);
      st7565_drawstring(buffer, 75, 5, "2.5",fontMode);
      st7565_drawstring(buffer, 108, 5, "3.3",fontMode);

  }
	  else if(displayMode==100){

      //st7565_drawstring(buffer, 0, 0, "\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\x1C\x1D\x1E\x1F",fontMode);
		st7565_clear_buffer(buffer);
		st7565_drawstring(buffer, 0, 0, " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~\x7F",fontMode);
	  }
  //Mesaj temporar dreapta jos de schimbare mod, suprascrie ce este sub el
  if(showDisplayModeOverlay) {
      if((HAL_GetTick() - displayModeChangeTime) >= 1250) {
          showDisplayModeOverlay = 0;
      } else {
          uint8_t rect_x = 80;
          uint8_t rect_y = 45;
          uint8_t rect_w = 47;
          uint8_t rect_h = 18;
          
          st7565_fillrect(buffer, rect_x, rect_y, rect_w, rect_h, 0);
          
          st7565_drawline(buffer, rect_x, rect_y, rect_x + rect_w, rect_y, 1);
          st7565_drawline(buffer, rect_x, rect_y, rect_x, rect_y + rect_h, 1);
          st7565_drawline(buffer, rect_x + rect_w, rect_y, rect_x + rect_w, rect_y + rect_h, 1);
          st7565_drawline(buffer, rect_x, rect_y + rect_h, rect_x + rect_w, rect_y + rect_h, 1);
          
          uint8_t overlay_text[20];
          sprintf(overlay_text, "Mode %d", displayMode);
          st7565_drawstring(buffer, rect_x + 2, rect_y/8 + 1, overlay_text, fontMode);

      }
  }
  st7565_write_buffer(buffer);
    osDelay(10);
  }
  /* USER CODE END displayVoltReadFunction */
}

/* USER CODE BEGIN Header_readButtonFunction */
/**
* @brief Function implementing the readButton thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readButtonFunction */
void readButtonFunction(void *argument)
{
  /* USER CODE BEGIN readButtonFunction */
  /* Infinite loop */
	flag=1;
  for(;;)
  {
    hadc.Instance->CHSELR = 1<<ADC_CHANNEL_0;
    if (HAL_ADC_Start(&hadc) == HAL_OK) {
      if (HAL_ADC_PollForConversion(&hadc, 4) == HAL_OK) {
        buttonReadRaw = HAL_ADC_GetValue(&hadc);
      } else {
        buttonReadRaw = 666;
      }
    } else {
      buttonReadRaw = 666;
    }
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
        
        if(buttonDebounceTimer >= 50) {
            buttonStatePrev = buttonState;
            buttonState = rawButtonState;
            buttonTransitionFlag = 0;
            buttonDebounceTimer = 0;
            
            if(!buttonTransitionFlag) {
                if(buttonState == 1 && buttonStatePrev == 0) {
                    if(displayMode==0)displayMode=displayModeMax;
                    else displayMode--;
                    showDisplayModeOverlay = 1;
                    displayModeChangeTime = HAL_GetTick();
                }
                else if(buttonState == 2 && buttonStatePrev == 0) {
                    if(displayMode==displayModeMax)displayMode=0;
                    else displayMode++;
                    showDisplayModeOverlay = 1;
                    displayModeChangeTime = HAL_GetTick();
                }
                else if(buttonState == 3 && buttonStatePrev == 0) {

                }
                else if(buttonState == 4 && buttonStatePrev == 0) {

                }
            }
        }
    } else {
        buttonTransitionFlag = 0;
        buttonDebounceTimer = 0;
    }
    
    osDelay(5);
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
