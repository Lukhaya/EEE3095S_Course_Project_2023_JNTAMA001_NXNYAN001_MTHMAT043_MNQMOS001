/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim3;

/*GLOBAL VARIABLES*/
uint32_t prev_millis = 0;
uint32_t curr_millis = 0;
uint32_t delay_t = 1000; // Initialise delay to 500ms
uint32_t adc_val;
uint32_t button_pressed = 0;
uint32_t binary_values[320];
uint32_t checkpoint[16];
uint32_t counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
void writeLCD(char *char_in);
uint32_t pollADC(void);
uint32_t ADCtoCCR(uint32_t adc_val);
void convertBinaryToString(uint32_t binary_values[16], char* result_string);
void handleCheckpointValue(void);
uint32_t binaryToInt(uint32_t startIndex);
void printAllValues(void);
uint32_t binaryToIntCheckpoint(uint32_t startIndex);
void writeLCD2(char *char_in);
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM3_Init();

  /* Initialize LCD */
  init_LCD();

  // PWM setup
  uint32_t CCR = 0;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Start PWM on TIM3 Channel 3


  char adcStringValue[20];
  writeLCD("Waiting...");  /* Initialize all configured peripherals */
  while(button_pressed == 0);
  writeLCD("Button pressed"); // Ready to receive
  adc_val = pollADC(); //takes adc_val
  uint32_t default_val = pollADC();

  /* Wait for the starting condition which is to turn on the LED before transmitting (i.e Start bit )*/
  while(adc_val > 200 ){adc_val = pollADC();}
  //start bit sent

  HAL_Delay(700);
  lcd_command(CLEAR);

  int prev_time = HAL_GetTick();
   uint32_t index = 0;
   int errorRate = 0;
   int currentTime = HAL_GetTick();

   //Here we loop until we get all the bits sent by the transmitter , including checkpoint

  while (1)
   {
 	//HAL_GPIO_WritePin(GPIOB, LED7_Pin, GPIO_PIN_RESET);

	/* binary_value is the current bit received from the transmitter ,*/
 	uint32_t binary_value = 0;
 	adc_val = pollADC();

 	/* Check if the bit sent is 1 or 0 */
 	if(adc_val < 200) {
 			binary_value = 1;
 			//HAL_GPIO_WritePin(GPIOB, LED7_Pin, GPIO_PIN_SET);
 	}
 	else binary_value = 0;


 	/* TODO :read  one bit every 1s/500ms ,after (1s/500ms) increment the index to read another value*/
 	if(HAL_GetTick() - prev_time <= 1000){

 		/*TODO: For every 100ms of (1s/500ms) check if the binary_value is changing
 		 *NOTE: This is to detect if the senderr is sending a stop codition
 		 */
 		if (HAL_GetTick()-currentTime > 100){
 			if(binary_values[index] != binary_value)errorRate++;
 			binary_values[index] = binary_value;
 			currentTime = HAL_GetTick();
 		}
 	}
 	/*(1s/500ms) has passed read another bit value  */
 	else{

 		//checks if stop condition was met
 		if (errorRate > 5)
 			{
 			counter = (index/16); //calc samples received
 			handleCheckpointValue();
 			//break out of while loop
 			break;
 			}
 		errorRate = 0;
 		 sprintf(adcStringValue,"%u",(int)binary_value);
 		 lcd_command(0x06);
 		 lcd_putstring(adcStringValue);
 		 index++;

 		prev_time = HAL_GetTick();
 	 	if((index)% 16==0){
 	 		int32_t value  = 0;
 	 		if(index == 0)value = binaryToInt(index);
 	 	    else  value = binaryToInt(index-16);
 	 		lcd_command(0xC0);
 	 		lcd_command(0xC0);

 	 		sprintf(adcStringValue,"%u", value);
 	 		lcd_putstring(adcStringValue);

 	 		lcd_command(0x00);
 	 		lcd_command(0x01);
 	 		counter++;
 	 	}

 	}

 	// Update PWM value; TODO: Get CRR
 	CCR = ADCtoCCR(adc_val);

 	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, CCR);
   }
   /* USER CODE END 3 */
 }

/**
  * @brief Compares the received and calculated counter
  * @retval None
  */
void handleCheckpointValue(){
	char binary_values_to_print[16];
	uint32_t index = 0;
	int startIndex = (counter-1)*16;
	uint32_t received_counter = binaryToInt(startIndex);
	sprintf(binary_values_to_print,"Counter %u", received_counter);
	writeLCD(binary_values_to_print);

    //TODO : Turn on LED in PIN for 2 seconds/Flash if counter is incorrect

	//code starts here
	if(received_counter == (counter-1)){
		HAL_GPIO_WritePin(GPIOB, LED7_Pin, GPIO_PIN_SET);
		HAL_Delay(2000);
		HAL_GPIO_WritePin(GPIOB, LED7_Pin, GPIO_PIN_RESET);
		printAllValues();
	}
	else{
		int inittime = HAL_GetTick();
		writeLCD("Error!!");
		while ((int) HAL_GetTick -inittime   < 2000){
			HAL_GPIO_TogglePin(GPIOB, LED7_Pin);
			HAL_Delay(100);
		}
		HAL_GPIO_WritePin(GPIOB, LED7_Pin, GPIO_PIN_RESET);
	}
	//code ends here
}

/**
  * @brief Prints all the samples received from the transmitter EXCEPT the Checkpoint
  * @retval None
  */
void printAllValues(void){
	char binary_values_to_print[16];

	//TODO : Print all values in the sample array
	int counterStartIndex = (counter-1)*16;
	uint32_t startIndex = 0;
	for(int i = 0 ; i < (counter-1) ; i++){
		uint32_t value = binaryToInt(startIndex);
		sprintf(binary_values_to_print,"sampleVal:%u", value);
		writeLCD(binary_values_to_print);
		startIndex += 16;
		HAL_Delay(2000);
	}
}

/**
  * @brief Convert binary values to decimal
  * @param startIndex
  * @retval uint32_t converted decimal value
  */
uint32_t binaryToInt(uint32_t startIndex) {
    uint32_t decimal = 0;
    for (int i = startIndex; i < startIndex+16; i++) {
        decimal = (decimal << 1) | binary_values[i];
    }

    return decimal;
}

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
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
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  ADC1->CR |= ADC_CR_ADCAL;
  while(ADC1->CR & ADC_CR_ADCAL);			// Calibrate the ADC
  ADC1->CR |= (1 << 0);						// Enable ADC
  while((ADC1->ISR & (1 << 0)) == 0);		// Wait for ADC ready
  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 47999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}
// Assuming the LCD display is capable of displaying strings
void convertBinaryToString(uint32_t binary_values[16], char* result_string) {
    result_string[0] = '\0'; // Initialize the result string as an empty string
    char temp[33]; // Temporary string to hold each binary value

    for (int i = 0; i < 16; i++) {
        sprintf(temp, "%u", binary_values[i]); // Convert the binary value to a string
        strcat(result_string, temp); // Concatenate the binary string to the main string
    }
}

void EXTI0_1_IRQHandler(void)
{
	curr_millis = HAL_GetTick();
	if (curr_millis - prev_millis >= 100) {  //Handles debouncing
			prev_millis = curr_millis;
			button_pressed = 1;
	    }
	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
}


void writeLCD(char *char_in){
    delay(3000);
	lcd_command(CLEAR);
	lcd_putstring(char_in);

}

uint32_t pollADC(void){
	HAL_ADC_Start(&hadc);
	adc_val = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	return adc_val;
}

// Calculate PWM CCR value
uint32_t ADCtoCCR(uint32_t adc_val){
  // TODO: Calculate CCR val using an appropriate equation
	uint32_t val = (adc_val*47999 ) / 4095;
	return val;
}

void ADC1_COMP_IRQHandler(void)
{
	adc_val = HAL_ADC_GetValue(&hadc); // read adc value
	HAL_ADC_IRQHandler(&hadc); //Clear flags
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT
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
