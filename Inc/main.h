/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdint.h"
#include "stm32l4xx.h"
#include <ucos_ii.h>
#include "filtering.h"
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define D_RST_Pin                    GPIO_PIN_3
#define D_RST_GPIO_Port              GPIOA
#define D_CS_Pin                     GPIO_PIN_4
#define D_CS_GPIO_Port               GPIOA
#define D_IRQ_Pin                    GPIO_PIN_0
#define D_IRQ_GPIO_Port              GPIOB
#define D_IRQ_EXTI_IRQn              EXTI0_IRQn
#define D_WAKEUP_Pin                 GPIO_PIN_1
#define D_WAKEUP_GPIO_Port           GPIOB
#define D_RS485_TX_Ctrl_Pin          GPIO_PIN_8
#define D_RS485_TX_Ctrl_GPIO_Port    GPIOA
#define LED1_PIN										 GPIO_PIN_14
#define LED1_PIN_Port								 GPIOC
#define LED3_PIN                		 GPIO_PIN_2
#define LED4_PIN                 		 GPIO_PIN_1
#define LED_PIN_Port                 GPIOA

#define LED_RED_ON     HAL_GPIO_WritePin(LED_PIN_Port,LED4_PIN,GPIO_PIN_RESET)
#define LED_RED_OFF    HAL_GPIO_WritePin(LED_PIN_Port,LED4_PIN,GPIO_PIN_SET)

#define RS485_TX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_SET)
#define RS485_RX       HAL_GPIO_WritePin(D_RS485_TX_Ctrl_GPIO_Port, D_RS485_TX_Ctrl_Pin, GPIO_PIN_RESET)

#define DECA_TAG               0;//标签
#define DECA_ANCHOR            1;//基站
#define DECA_REMOTECONTROL     2;//遥控

/*任务优先级定义*/
#define UWB_MODECHOOSE_PRIO      5
#define UWB_TAG_PRIO             6
#define UWB_ANCHOR_PRIO          7
#define UWB_REMOTECONTROL_PRIO   8
#define USART1_RXDATA_PRIO       9
#define USART1_TXDATA_PRIO       10
#define LED_BLINK_PRIO           27
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
