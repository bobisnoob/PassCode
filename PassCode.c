/**
  ******************************************************************************
  * File Name     : PassCode.c
  * PROJECT       : PROG8125 -Assignment #2
  * PROGRAMMER    : Abdelraheem Alkuor (Abdel)
  * FIRST VERSION : 2016-10-16
  * Description   : This program takes accessCode from USART1 and compares it to
  * 				saved access codes.If they match, "Access granted" is printed,
  * 				out on serial port terminal, "Access granted" sound is produced,
  * 				by calling accessGrantedSound function,on PA04 and green LED
  * 				is turned on by calling accessGrantedLED.If they don't match,
  * 				"Access denied"	is printed out on serial port terminal,
  * 				"Access denied" sound is produced, by calling accessdeniedSound
  * 				function, on PA04 and red LED is turned on  by calling
  * 				accessDeniedLED function.
  *
  * 			    The program uses USART1(ST-LINK) received data in interrupt mode, and
  * 			    DAC(PA04) to output audio.
  *
  * 			    Part of the program is written in Interrupt Service Routines
  * 			    (stm32f3xx_it.c) under "void USART1_IRQHandler(void)" fucntion
  *
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private Constants and defines ---------------------------------------------------------*/
#define DENID_DATA_SAMPLES_SIZE 	8266
#define GRANTED_DATA_SAMPLES_SIZE 	10031
#define TRANSMIT_DATA_SIZE 			1
#define DENID						0
#define GRANTED						1
#define ACCESS_STATE_UNKNOWN		2
#define RECEIVED_CODE_BYTE_SIZE		1
#define TIME_OUT					100
extern const uint16_t accessDeniedData[DENID_DATA_SAMPLES_SIZE];
extern const uint16_t accessGrantedData[GRANTED_DATA_SAMPLES_SIZE];
/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern char receivedCodeByte[RECEIVED_CODE_BYTE_SIZE];
extern volatile int  accessState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
void accessDeniedSound(void);
void accessGrantedSound(void);
void accessGrantedLED(void);
void accessDenidLED(void);

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC_Init();
  MX_USART1_UART_Init();


  /* Receive an amount of data in interrupt mode */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)receivedCodeByte, RECEIVED_CODE_BYTE_SIZE);

  /* Enables DAC and starts conversion of channel */
  HAL_DAC_Start( &hdac, DAC_CHANNEL_1);

  HAL_UART_Transmit(&huart1, (uint8_t *)"Enter Access code:\r\n", 20, TIME_OUT);//20 is the size of "Enter Access code:\r\n"
  /* Infinite loop */
  while (1)
  {
	  if( accessState == DENID )
	  {
		  accessDenidLED(); // Red LED is turned on
		  accessDeniedSound();
		  accessState = ACCESS_STATE_UNKNOWN;
	  }

	  if( accessState == GRANTED )
	  {
		  accessGrantedLED(); // Green LED is turned on
		  accessGrantedSound();
		  accessState = ACCESS_STATE_UNKNOWN;
	  }
  }

}

/*
 * FUNCTION      : accessDeniedSound
 * DESCRIPTION   : This function produces sound which is "Access denied" when it is called by using
 * 				   DAC peripheral (PA04)
 * PARAMETERS    : Void. It doesn't take any parameters
 *
*/
void accessDeniedSound(void)
{
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000);// Timer counter is set to 1us to be able to reproduce the sound
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	for(int i=0 ; i<DENID_DATA_SAMPLES_SIZE ; i++) // converting digital values to analog values and send them to PA04
	{
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, accessDeniedData[i]);
		HAL_Delay(110); // a spaced time interval in us between each DAC value to reproduce audible sound
	}
}

/*
 * FUNCTION      : accessGrantedSound
 * DESCRIPTION   : This function produces sound which is "Access granted" when it is called using
 * 				   DAC peripheral (PA04)
 * PARAMETERS    : Void. It doesn't take any parameters
 *
*/
void accessGrantedSound(void)
{
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000000); // Timer counter is set to 1us to be able to reproduce the sound
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	for(int i=0 ; i<GRANTED_DATA_SAMPLES_SIZE ; i++) // converting digital values to analog values and send them to PA04
	{
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, accessGrantedData[i]);
		HAL_Delay(110); // a spaced time interval in us between each DAC value to reproduce audible sound
	}
}

/*
 * FUNCTION      : accessDenidLED
 * DESCRIPTION   : This function turns red LED on PIN_E9 and turns off green LED on PIN_E15
 * PARAMETERS    : Void. It doesn't take any parameters
 *
*/
void accessDenidLED(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); 	// Turn red LED on
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);  //Turn green LED off
}

/*
 * FUNCTION      : accessGrantedLED
 * DESCRIPTION   : This function turns a green LED on PIN_E15 and turns off red LED on PIN_E9
 * PARAMETERS    : Void. It doesn't take any parameters
 *
*/
void accessGrantedLED(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // Turn red LED off
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);  //Turn green LED on

}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE9 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
