/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/*// ----------------------------------------------------------------------------
**
**  Abstract: main program
**
**  Goal: Read the Accelerometer from ADXL345 Sensor and display it on USART and on
**  the in-built LTDC.
**
**	Author: Chinmay G
**
**	Organization: None

// ----------------------------------------------------------------------------
 */

//---------------------INCLUDES-----------------------------//
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_gpio_ex.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_conf.h"


//------------------MACRO DEFINITIONS-----------------------//
#define LED_PORT													(GPIOG)
#define GREENLED_PIN_NUMBER              	(GPIO_PIN_13)
#define REDLED_PIN_NUMBER              	  (GPIO_PIN_14)

//-------------------Configurations------------------------//

GPIO_InitTypeDef GPIO_InitStructure;
USART_HandleTypeDef USART1Handle;


//---------------Private Function Definitions--------------//

void CPG_LED_Init(void)
{
	__HAL_RCC_GPIOG_CLK_ENABLE();			//GPIO Port G Clock init as the leds are connected to Port G pins

	GPIO_InitStructure.Pin = GREENLED_PIN_NUMBER | REDLED_PIN_NUMBER;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull = GPIO_PULLUP;

	HAL_GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

void CPG_USART_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();			//GPIO Port A Clock init as UART pins are on port A
	__HAL_RCC_USART1_CLK_ENABLE();		//USART 1 Clock init

	/* GPIO Initialization*/

	GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_10 ; // UART TX RX pins
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;					// ALternet Function
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Alternate = GPIO_AF7_USART1;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART 1 Initialization*/

	USART1Handle.Instance = USART1;
	USART1Handle.Init.BaudRate = 115200;
	USART1Handle.Init.Mode = USART_MODE_TX;
	USART1Handle.Init.StopBits = USART_STOPBITS_1;
	USART1Handle.Init.WordLength = USART_WORDLENGTH_8B;
	USART1Handle.Init.Parity = USART_PARITY_NONE;

	HAL_USART_Init(&USART1Handle);
}



//--------------------Main Function-----------------------//
int main()
{
  // Initialise the HAL Library; it must be the first function
  // to be executed before the call of any HAL function.
  HAL_Init();

  //Initialise Green and red leds.
  CPG_LED_Init();
  //Initialise USART1
  CPG_USART_Init();

  char buffer[] = "USART Transmission\r\n";

	while(1)
		{
			HAL_GPIO_WritePin(LED_PORT, GREENLED_PIN_NUMBER , GPIO_PIN_SET);
			HAL_USART_Transmit(&USART1Handle, &buffer, sizeof(buffer), HAL_MAX_DELAY);
			HAL_Delay(500);
			HAL_GPIO_WritePin(LED_PORT, GREENLED_PIN_NUMBER, GPIO_PIN_RESET);
			HAL_Delay(500);
		}

	return 0;
}




//SysTick Handler is required for HAL_Delay() Function

//---------SYSTICK HANDLER----------------//
void SysTick_Handler(void)
{
    HAL_IncTick();
}
