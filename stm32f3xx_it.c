/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
#include <stdio.h>
#include <string.h>
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
#define TRANSMIT_DATA_SIZE 			1
#define DENID						0
#define GRANTED						1
#define ACCESS_STATE_UNKNOWN		2
#define ENTER						13
#define RECEIVED_CODE_BYTE_SIZE		1
#define RECEIVED_ACCESS_CODE_SIZE	5
#define ACCESS_CODE_ROW_SIZE		10
#define ACCESS_CODE_COL_SIZE		5
#define TIME_OUT					100
#define RECEIVED_CODE_FIRST_BYTE	0
#define COMPARISON_PASSED			0


/* Access Code PINS */
#define ACCESS_CODE_PIN0			"0000"
#define ACCESS_CODE_PIN1			"1111"
#define ACCESS_CODE_PIN2			"2222"
#define ACCESS_CODE_PIN3			"3333"
#define ACCESS_CODE_PIN4			"4444"
#define ACCESS_CODE_PIN5			"5555"
#define ACCESS_CODE_PIN6			"6666"
#define ACCESS_CODE_PIN7			"7777"
#define ACCESS_CODE_PIN8			"8888"
#define ACCESS_CODE_PIN9			"9999"

char receivedCodeByte[RECEIVED_CODE_BYTE_SIZE];
char receivedAccessCode[RECEIVED_ACCESS_CODE_SIZE];

char accessCode[ACCESS_CODE_ROW_SIZE][ACCESS_CODE_COL_SIZE]= {
																ACCESS_CODE_PIN0,
																ACCESS_CODE_PIN1,
																ACCESS_CODE_PIN2,
																ACCESS_CODE_PIN3,
																ACCESS_CODE_PIN4,
																ACCESS_CODE_PIN5,
																ACCESS_CODE_PIN6,
																ACCESS_CODE_PIN7,
																ACCESS_CODE_PIN8,
																ACCESS_CODE_PIN9
															 };
volatile int  accessState = ACCESS_STATE_UNKNOWN;
int i=0; //initialize counter increment
uint8_t receivedAccessCodeLength;
/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
*/
void USART1_IRQHandler(void)
{

	HAL_UART_IRQHandler(&huart1);

  	if(receivedCodeByte[RECEIVED_CODE_FIRST_BYTE] != ENTER) // check if enter is pressed
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)receivedCodeByte, TRANSMIT_DATA_SIZE, TIME_OUT);//Echo the received data
		receivedAccessCode[i] = receivedCodeByte[RECEIVED_CODE_FIRST_BYTE]; //Store incoming byte to receivedAccessCode
		i++;
    }
	else
	{
		receivedAccessCode[i] = 0; //Terminate the stored code with NULL
		HAL_UART_Transmit(&huart1, (uint8_t *)"\n", TRANSMIT_DATA_SIZE,TIME_OUT);
		HAL_UART_Transmit(&huart1, (uint8_t *)"\r", TRANSMIT_DATA_SIZE,TIME_OUT);
		accessState = DENID; //Assume accessState is DENID

		for(i=0; i<ACCESS_CODE_ROW_SIZE; i++)
		{
			if ( strcmp(receivedAccessCode, accessCode[i]) == COMPARISON_PASSED) //Compare the entered access code with the stored
			{																     //access codes
				HAL_UART_Transmit(&huart1, (uint8_t *)"Access Granted\n\r", 16, TIME_OUT); //16 is the size of "Access Granted\n\r"
				accessState=GRANTED	;
  			}
		}

		if( accessState == DENID )
		{
			HAL_UART_Transmit(&huart1, (uint8_t *)"Access Denied\n\rTry again\n\r", 26, TIME_OUT); //26 is the size of
  		}																						   //"Access Denied\n\rTry again\n\r"

		receivedAccessCodeLength = strlen(receivedAccessCode);// Get the receivedAccessCode length

		for( i=0; i<receivedAccessCodeLength; i++ ) // re-initialize the receivedAccessCode
		{
			receivedAccessCode[i] = 0;
    	}
    	  i=0;
	  }
  	//Enable receive interrupt
  	HAL_UART_Receive_IT(&huart1, (uint8_t *)receivedCodeByte,RECEIVED_CODE_BYTE_SIZE);

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
