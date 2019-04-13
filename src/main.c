/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
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

------------------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>

#include "main.h"

/* Private variables ---------------------------------------------------------*/

uint8_t buffer[6];
uint8_t devID = 0;
int16_t x,y,z;
float xg, yg, zg;
char x_char[3], y_char[3], z_char[3];

/* Private Functions ---------------------------------------------------------*/

void CPG_USART_Init(void);
void CPG_I2C_Init(void);
void i2c_write (uint8_t reg, uint8_t value);
void i2c_read_values (uint8_t reg, uint8_t noOfBytes);
void i2c_read_address (uint8_t reg);
void ADXL345_Init(void);

//--------------------Main Function-----------------------//
int main()
{
	// Private variables
	char output[32] = {};
	char newline = '\n';
  // Initialise the HAL Library; it must be the first function
  // to be executed before the call of any HAL function.
  HAL_Init();

  //Initialise USART1
  CPG_USART_Init();
  // Initialise I2C
  CPG_I2C_Init();

  ADXL345_Init();

	while(1)
		{
			i2c_read_values(0x32, 6);
		  x = ((buffer[1]<<8)|buffer[0]);
		  y = ((buffer[3]<<8)|buffer[2]);
		  z = ((buffer[5]<<8)|buffer[4]);

		  sprintf(&output[0],"x:%d y:%d z:%d\r\n", x,y,z);
			HAL_USART_Transmit(&USART1Handle, &output[0], sizeof(output), 100);

			HAL_Delay(1000);
		}

	return 0;
}




//SysTick Handler is required for HAL_Delay() Function

//---------SYSTICK HANDLER----------------//
void SysTick_Handler(void)
{
    HAL_IncTick();
}


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
	__HAL_RCC_GPIOB_CLK_ENABLE();			//GPIO Port B Clock init as I2C pins are on port A

	/* GPIO Initialization*/
	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9 ; // I2C pins
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;					// Alternet Function
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;

	__HAL_RCC_I2C1_CLK_ENABLE();

  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* I2C 1 Initialization*/
  I2C1Handle.Instance = I2C1;
  I2C1Handle.Init.ClockSpeed = 100000;
  I2C1Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C1Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2C1Handle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  I2C1Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2C1Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  I2C1Handle.Init.OwnAddress1 = 0;
  I2C1Handle.Init.OwnAddress2 = 0;

  HAL_I2C_Init(&I2C1Handle);

}

void i2c_write (uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit(&I2C1Handle, I2C_ADDRESS, &data[0], 2, 100);
}

void i2c_read_values (uint8_t reg, uint8_t noOfBytes)
{
	HAL_I2C_Mem_Read (&I2C1Handle, I2C_ADDRESS, reg, 1, buffer, noOfBytes, 100);
}

void i2c_read_address (uint8_t reg)
{
	HAL_I2C_Mem_Read (&I2C1Handle, I2C_ADDRESS, reg, 1, &devID, 1, 1000);
}

void ADXL345_Init(void)
{
		i2c_read_address (0x00); // read the DEVID
		i2c_write(0x31, 0x01);
		i2c_write (0x2D, 0x00);  // reset all bits
		i2c_write (0x2D, 0x08);  // power_cntl measure and wake up 8hz
}
