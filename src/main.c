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
#define I2C_ADDRESS 0x53

//-------------------Configurations------------------------//

GPIO_InitTypeDef GPIO_InitStructure;
USART_HandleTypeDef USART1Handle;
I2C_HandleTypeDef  I2C1Handle;


//---------------Private Function Definitions--------------//
void CPG_USART_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();			//GPIO Port A Clock init as UART pins are on port A
	__HAL_RCC_USART1_CLK_ENABLE();		//USART 1 Clock init

	/* GPIO Initialization*/

	GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_10 ; // UART TX RX pins
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;					// Alternet Function
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


void CPG_I2C_Init(void)
{
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();			//GPIO Port B Clock init as I2C pins are on port A

	/* GPIO Initialization*/
	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9 ; // UART TX RX pins
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;					// Alternet Function
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* I2C 1 Initialization*/
  I2C1Handle.Instance = I2C1;
  I2C1Handle.Init.ClockSpeed = 50000;
  I2C1Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C1Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2C1Handle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2C1Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2C1Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  I2C1Handle.Init.OwnAddress1 = I2C_ADDRESS;

  HAL_I2C_Init(&I2C1Handle);

}

//--------------------Main Function-----------------------//
int main()
{
  // Initialise the HAL Library; it must be the first function
  // to be executed before the call of any HAL function.
  HAL_Init();

  //Initialise USART1
  CPG_USART_Init();
  CPG_I2C_Init();

  char firststr[] = "Transmission Started\r\n";
  HAL_USART_Transmit(&USART1Handle, &firststr, sizeof(firststr), HAL_MAX_DELAY);
  char buffer[32] = {};
	while(1)
		{
			HAL_I2C_Master_Receive(&I2C1Handle, I2C_ADDRESS, &buffer, sizeof(buffer), HAL_MAX_DELAY);
			HAL_Delay(100);
			HAL_USART_Transmit(&USART1Handle, &buffer, sizeof(buffer), HAL_MAX_DELAY);
			HAL_Delay(900);
		}

	return 0;
}




//SysTick Handler is required for HAL_Delay() Function

//---------SYSTICK HANDLER----------------//
void SysTick_Handler(void)
{
    HAL_IncTick();
}

