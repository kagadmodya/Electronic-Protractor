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
#include <math.h>

#include "main.h"

/* Private variables ---------------------------------------------------------*/

uint8_t buffer[6];
uint8_t devID = 0;
//store Previous and new values of x, y, z
int16_t x_raw = 0, y_raw = 0, z_raw = 0;
int16_t xP = 0;// yP = 0, zP = 0;
int16_t x_deg = 0;// y_deg = 0, z_deg = 0;
int16_t theta = 0;



/* Private Functions ---------------------------------------------------------*/

void CPG_GPIO_Init(void);
void CPG_USART_Init(void);
void CPG_I2C_Init(void);
void configureEXTI(void);
void i2c_write (uint8_t reg, uint8_t value);
void i2c_read_values (uint8_t reg, uint8_t noOfBytes);
void i2c_read_address (uint8_t reg);
void ADXL345_Init(void);

//--------------------Main Function-----------------------//
int main()
{
	// Private variables
	char output[32] = {};

  // Initialise the HAL Library; it must be the first function
  // to be executed before the call of any HAL function.
  HAL_Init();
  configureEXTI();
  CPG_GPIO_Init(); 	// Initialise GPIO for required peripherals
  CPG_USART_Init();	// Initialise USART1
  CPG_I2C_Init();		// Initialise I2C

  ADXL345_Init();

	while(1)
		{
			i2c_read_values(0x32, 6);

		  x_raw = ((buffer[1]<<8)|buffer[0]);
//		  y_raw = ((buffer[3]<<8)|buffer[2]);
		  z_raw = ((buffer[5]<<8)|buffer[4]);

		  int(x_deg) = (atan2(x_raw,z_raw) * 180.0) / M_PI;
		  theta = x_deg - xP;
//		  theta = x_raw - xP;
		  sprintf(&output[0],"Angle: %d\r\n", theta);
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

void CPG_GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE(); //For usart pins and user button pin
	__HAL_RCC_GPIOB_CLK_ENABLE(); //For i2c pins

	//GPIO for USART1
	GPIO_InitStructure.Pin 						= GPIO_PIN_9 | GPIO_PIN_10 ; // UART TX RX pins
	GPIO_InitStructure.Mode 					= GPIO_MODE_AF_PP;					// Alternet Function
  GPIO_InitStructure.Speed 					= GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull 					= GPIO_PULLUP;
  GPIO_InitStructure.Alternate 			= GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	// GPIO for User Buttons
	GPIO_InitStructure.Pin 						= GPIO_PIN_0;
	GPIO_InitStructure.Mode 					= GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Speed 					= GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull 					= GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIO for i2c1
	GPIO_InitStructure.Pin 						= GPIO_PIN_8 | GPIO_PIN_9 ; // SCL-PB8, SDA-PB9
	GPIO_InitStructure.Mode 					= GPIO_MODE_AF_OD;					// Alternet Function
  GPIO_InitStructure.Speed 					= GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Pull					 	= GPIO_PULLUP;
  GPIO_InitStructure.Alternate 			= GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void CPG_USART_Init(void)
{
	__HAL_RCC_USART1_CLK_ENABLE();
	USART1Handle.Instance 					= USART1;
	USART1Handle.Init.BaudRate 			= 115200;
	USART1Handle.Init.Mode 					= USART_MODE_TX;
	USART1Handle.Init.StopBits 			= USART_STOPBITS_1;
	USART1Handle.Init.WordLength 		= USART_WORDLENGTH_8B;
	USART1Handle.Init.Parity 				= USART_PARITY_NONE;
	HAL_USART_Init(&USART1Handle);
}

void CPG_I2C_Init(void)
{
	__HAL_RCC_I2C1_CLK_ENABLE();
  I2C1Handle.Instance 						= I2C1;
  I2C1Handle.Init.ClockSpeed 			= 100000;
  I2C1Handle.Init.AddressingMode 	= I2C_ADDRESSINGMODE_7BIT;
  I2C1Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2C1Handle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  I2C1Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2C1Handle.Init.NoStretchMode 	= I2C_NOSTRETCH_DISABLE;
  I2C1Handle.Init.OwnAddress1 		= 0;
  I2C1Handle.Init.OwnAddress2 		= 0;
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


void configureEXTI(void)
{
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	char intbuff[] = "Interrupt\r\n";
	HAL_USART_Transmit(&USART1Handle, &intbuff[0], 12, 100);
	xP = x_deg;
}
