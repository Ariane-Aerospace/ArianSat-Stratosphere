/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define GPIO_Pin GPIO_PIN_2
#define GPIO_GPIO_Port GPIOE
#define GPIOE3_Pin GPIO_PIN_3
#define GPIOE3_GPIO_Port GPIOE
#define GPIOE4_Pin GPIO_PIN_4
#define GPIOE4_GPIO_Port GPIOE
#define GPIOE5_Pin GPIO_PIN_5
#define GPIOE5_GPIO_Port GPIOE
#define GPIOE6_Pin GPIO_PIN_6
#define GPIOE6_GPIO_Port GPIOE
#define GPIOF2_Pin GPIO_PIN_2
#define GPIOF2_GPIO_Port GPIOF
#define ADC_GPIO_Pin GPIO_PIN_3
#define ADC_GPIO_GPIO_Port GPIOF
#define ADC_GPIOF4_Pin GPIO_PIN_4
#define ADC_GPIOF4_GPIO_Port GPIOF
#define ADC_GPIOF5_Pin GPIO_PIN_5
#define ADC_GPIOF5_GPIO_Port GPIOF
#define ADC_GPIOF6_Pin GPIO_PIN_6
#define ADC_GPIOF6_GPIO_Port GPIOF
#define ADC_GPIOF7_Pin GPIO_PIN_7
#define ADC_GPIOF7_GPIO_Port GPIOF
#define ADC_GPIOF8_Pin GPIO_PIN_8
#define ADC_GPIOF8_GPIO_Port GPIOF
#define ADC_GPIOF9_Pin GPIO_PIN_9
#define ADC_GPIOF9_GPIO_Port GPIOF
#define ADC_GPIOF10_Pin GPIO_PIN_10
#define ADC_GPIOF10_GPIO_Port GPIOF
#define ADC_GPIOC0_Pin GPIO_PIN_0
#define ADC_GPIOC0_GPIO_Port GPIOC
#define ADC_GPIOC1_Pin GPIO_PIN_1
#define ADC_GPIOC1_GPIO_Port GPIOC
#define ADC_GPIOC2_Pin GPIO_PIN_2
#define ADC_GPIOC2_GPIO_Port GPIOC
#define ADC_GPIOC3_Pin GPIO_PIN_3
#define ADC_GPIOC3_GPIO_Port GPIOC
#define ADC_GPIOA4_Pin GPIO_PIN_4
#define ADC_GPIOA4_GPIO_Port GPIOA
#define SPI1_SCK_ALT_Pin GPIO_PIN_5
#define SPI1_SCK_ALT_GPIO_Port GPIOA
#define SPI1_MISO_ALT_Pin GPIO_PIN_6
#define SPI1_MISO_ALT_GPIO_Port GPIOA
#define SPI1_MOSI_ALT_Pin GPIO_PIN_7
#define SPI1_MOSI_ALT_GPIO_Port GPIOA
#define ADC_GPIOC4_Pin GPIO_PIN_4
#define ADC_GPIOC4_GPIO_Port GPIOC
#define ADC_GPIOC5_Pin GPIO_PIN_5
#define ADC_GPIOC5_GPIO_Port GPIOC
#define ADC_GPIOB0_Pin GPIO_PIN_0
#define ADC_GPIOB0_GPIO_Port GPIOB
#define ADC_GPIOB1_Pin GPIO_PIN_1
#define ADC_GPIOB1_GPIO_Port GPIOB
#define GPIOB2_Pin GPIO_PIN_2
#define GPIOB2_GPIO_Port GPIOB
#define GPIOF11_Pin GPIO_PIN_11
#define GPIOF11_GPIO_Port GPIOF
#define GPIOF12_Pin GPIO_PIN_12
#define GPIOF12_GPIO_Port GPIOF
#define GPIOF13_Pin GPIO_PIN_13
#define GPIOF13_GPIO_Port GPIOF
#define GPIOF14_Pin GPIO_PIN_14
#define GPIOF14_GPIO_Port GPIOF
#define GPIOF15_Pin GPIO_PIN_15
#define GPIOF15_GPIO_Port GPIOF
#define GPIOG0_Pin GPIO_PIN_0
#define GPIOG0_GPIO_Port GPIOG
#define GPIOG1_Pin GPIO_PIN_1
#define GPIOG1_GPIO_Port GPIOG
#define GPIOE7_Pin GPIO_PIN_7
#define GPIOE7_GPIO_Port GPIOE
#define GPIOE8_Pin GPIO_PIN_8
#define GPIOE8_GPIO_Port GPIOE
#define GPIOE9_Pin GPIO_PIN_9
#define GPIOE9_GPIO_Port GPIOE
#define GPIOE10_Pin GPIO_PIN_10
#define GPIOE10_GPIO_Port GPIOE
#define GPIOE11_Pin GPIO_PIN_11
#define GPIOE11_GPIO_Port GPIOE
#define GPIOE12_Pin GPIO_PIN_12
#define GPIOE12_GPIO_Port GPIOE
#define GPIOE13_Pin GPIO_PIN_13
#define GPIOE13_GPIO_Port GPIOE
#define GPIOE14_Pin GPIO_PIN_14
#define GPIOE14_GPIO_Port GPIOE
#define GPIOE15_Pin GPIO_PIN_15
#define GPIOE15_GPIO_Port GPIOE
#define I2C2_SCL_ALT_SPI2_SCK_Pin GPIO_PIN_10
#define I2C2_SCL_ALT_SPI2_SCK_GPIO_Port GPIOB
#define I2C2_SDA_ALT_Pin GPIO_PIN_11
#define I2C2_SDA_ALT_GPIO_Port GPIOB
#define GPIOD10_Pin GPIO_PIN_10
#define GPIOD10_GPIO_Port GPIOD
#define GPIOD11_Pin GPIO_PIN_11
#define GPIOD11_GPIO_Port GPIOD
#define GPIOD12_Pin GPIO_PIN_12
#define GPIOD12_GPIO_Port GPIOD
#define GPIOD13_Pin GPIO_PIN_13
#define GPIOD13_GPIO_Port GPIOD
#define GPIOD14_Pin GPIO_PIN_14
#define GPIOD14_GPIO_Port GPIOD
#define GPIOD15_Pin GPIO_PIN_15
#define GPIOD15_GPIO_Port GPIOD
#define GPIOG3_Pin GPIO_PIN_3
#define GPIOG3_GPIO_Port GPIOG
#define GPIOG4_Pin GPIO_PIN_4
#define GPIOG4_GPIO_Port GPIOG
#define GPIOG5_Pin GPIO_PIN_5
#define GPIOG5_GPIO_Port GPIOG
#define GPIOG6_Pin GPIO_PIN_6
#define GPIOG6_GPIO_Port GPIOG
#define GPIOG7_Pin GPIO_PIN_7
#define GPIOG7_GPIO_Port GPIOG
#define GPIOG8_Pin GPIO_PIN_8
#define GPIOG8_GPIO_Port GPIOG
#define GPIOA15_Pin GPIO_PIN_15
#define GPIOA15_GPIO_Port GPIOA
#define CAn1_RX_ALT_Pin GPIO_PIN_0
#define CAn1_RX_ALT_GPIO_Port GPIOD
#define CAN1_TX_ALT_Pin GPIO_PIN_1
#define CAN1_TX_ALT_GPIO_Port GPIOD
#define GPIOD3_Pin GPIO_PIN_3
#define GPIOD3_GPIO_Port GPIOD
#define GPIOD4_Pin GPIO_PIN_4
#define GPIOD4_GPIO_Port GPIOD
#define UART2_TX_ALT_Pin GPIO_PIN_5
#define UART2_TX_ALT_GPIO_Port GPIOD
#define UART2_RX_ALT_Pin GPIO_PIN_6
#define UART2_RX_ALT_GPIO_Port GPIOD
#define GPIOD7_Pin GPIO_PIN_7
#define GPIOD7_GPIO_Port GPIOD
#define UART6_RX_ALT_Pin GPIO_PIN_9
#define UART6_RX_ALT_GPIO_Port GPIOG
#define GPIOG10_Pin GPIO_PIN_10
#define GPIOG10_GPIO_Port GPIOG
#define GPIOG11_Pin GPIO_PIN_11
#define GPIOG11_GPIO_Port GPIOG
#define GPIOG12_Pin GPIO_PIN_12
#define GPIOG12_GPIO_Port GPIOG
#define GPIOG13_Pin GPIO_PIN_13
#define GPIOG13_GPIO_Port GPIOG
#define UART6_TX_ALT_Pin GPIO_PIN_14
#define UART6_TX_ALT_GPIO_Port GPIOG
#define GPIOG15_Pin GPIO_PIN_15
#define GPIOG15_GPIO_Port GPIOG
#define GPIOE0_Pin GPIO_PIN_0
#define GPIOE0_GPIO_Port GPIOE
#define GPIOE1_Pin GPIO_PIN_1
#define GPIOE1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
