
  /// @file           : main.h
  /// @brief          : Header for main.c file.
  /// @attention

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f1xx_hal.h"

void Error_Handler(void);


#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BluetoothReset_Pin GPIO_PIN_14
#define BluetoothReset_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif
