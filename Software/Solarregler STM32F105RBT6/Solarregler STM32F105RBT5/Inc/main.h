/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_4
#define LD2_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_5
#define LD1_GPIO_Port GPIOA
#define DRDY_1_Pin GPIO_PIN_6
#define DRDY_1_GPIO_Port GPIOA
#define CS_1_Pin GPIO_PIN_7
#define CS_1_GPIO_Port GPIOA
#define DRDY_2_Pin GPIO_PIN_4
#define DRDY_2_GPIO_Port GPIOC
#define CS_2_Pin GPIO_PIN_5
#define CS_2_GPIO_Port GPIOC
#define DRDY_3_Pin GPIO_PIN_0
#define DRDY_3_GPIO_Port GPIOB
#define CS_3_Pin GPIO_PIN_1
#define CS_3_GPIO_Port GPIOB
#define C7_Pin GPIO_PIN_7
#define C7_GPIO_Port GPIOC
#define C6_Pin GPIO_PIN_8
#define C6_GPIO_Port GPIOC
#define C5_Pin GPIO_PIN_9
#define C5_GPIO_Port GPIOC
#define C4_Pin GPIO_PIN_8
#define C4_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_9
#define C3_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_10
#define C2_GPIO_Port GPIOA
#define C1_Pin GPIO_PIN_11
#define C1_GPIO_Port GPIOA
#define C0_Pin GPIO_PIN_12
#define C0_GPIO_Port GPIOA

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define MAX_ADJUSTABLE_TEMPERATURE 100.0
#define MIN_ADJUSTABLE_TEMPERATURE 0.0

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
