 /**
  ******************************************************************************
  * @file    bsp.h
  * @author  Microcontroller Division
  * @version V1.0.3
  * @date    May-2013
  * @brief   Input/Output defines
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H
#define __BSP_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"  
#include <cpu.h>
#include <stdio.h>

#define FALSE 0
#define TRUE !FALSE

/* MACROs for SET, RESET or TOGGLE Output port */

#define USART1baud_rate	115200 
#define USART2baud_rate	115200 //debug
#define USART3baud_rate	9600 //gps

typedef struct
{      
	char Data[200];
	CPU_INT16U DataLength;
}GprsDataTypeDef;

enum SystemState {
	Onetime,
	Cyclic,
	Alwayson,
	Shutdown
};

void  BSP_Uart1_Init (CPU_INT32U  baud_rate);
void  BSP_Uart2_Init (CPU_INT32U  baud_rate);
void  BSP_Uart3_Init (CPU_INT32U  baud_rate);

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
