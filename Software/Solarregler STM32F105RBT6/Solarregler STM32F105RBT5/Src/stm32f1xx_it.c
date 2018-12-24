/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */

#include "MAX31865.h"
#include "string.h"

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

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
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles CAN1 RX0 interrupt.
*/
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

	extern MAX31865_TypeDef MAXSolar;
	extern MAX31865_TypeDef MAXSens3;
	extern MAX31865_TypeDef MAXBuffer;
	
	extern float temperaturSolar;
	extern float temperaturSens3;
	extern float temperaturSolarMean;
	extern float temperaturBuffer;
	extern float temperaturBufferMean;
	
	extern float temperaturBufferMax;
	extern float temperaturSolarMin;
	
	extern float hysteresisON;
	extern float hysteresisOFF;
	
	extern uint8_t relaisState;
	extern uint8_t controllerActivated;
	extern uint8_t sens3Used;
	extern uint8_t sens3Solar;
	extern uint8_t sens3Buffer;
	
	extern uint16_t CanBaseAdress;
	
	static uint8_t relaisStateOld;
	//static float totalMean = 0.0;
	//static float meanArray[20] = {0};
	
	extern RTC_HandleTypeDef hrtc;
	extern int onTime;
	static RTC_TimeTypeDef tmpTime;
	
	
	
	HAL_GPIO_TogglePin(GPIOA, 0x1 << 5);
	
	relaisStateOld = relaisState;
	

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
	checkForFaults(&MAXSolar);
	checkForFaults(&MAXBuffer);
	
	//read temperatur sensors
	temperaturSolar = measureTemperatureOneShotConverted(&MAXSolar);
	temperaturBuffer = measureTemperatureOneShotConverted(&MAXBuffer);
	
	if(sens3Used){
		checkForFaults(&MAXSens3);
		temperaturSens3 = measureTemperatureOneShotConverted(&MAXSens3);
		
		if(sens3Buffer && !sens3Solar){
		temperaturBufferMean = (temperaturBuffer + temperaturSens3) / 2.0;
		} else {
		if(sens3Solar && !sens3Buffer){
		temperaturSolarMean = (temperaturSolar + temperaturSens3) / 2.0;
		}
	}
	}
	
	
	
	//create message for uart
	//sprintf(Message, "Temperatur Solar: %.2f \nTemperatur Puffer: %.2f \nStatus Pumpe: %d \n", temperaturSolar, temperaturBuffer, relaisState);
	//transmit data via uart
	//HAL_UART_Transmit(&huart2, (uint8_t*) Message, strlen(Message), HAL_MAX_DELAY);
	
	//decide to switch on the pump
if(controllerActivated){
if(sens3Used == 0){
	if(relaisState == 0){
		if(( temperaturSolar >= temperaturBuffer + hysteresisON) && (temperaturSolar >= temperaturSolarMin) && (temperaturBuffer <= temperaturBufferMax)){
			relaisState = 1;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);
		} else {
			relaisState = 0;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
	}
}
	if(relaisState == 1){
		if ((temperaturBuffer >= temperaturSolar + hysteresisOFF) || (temperaturSolar < temperaturSolarMin) || (temperaturBuffer > temperaturBufferMax)){
			relaisState = 0;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
		} else {
			relaisState = 1;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);		
		}
	}
} else {	

	if(sens3Buffer && !sens3Solar){	
	if(relaisState == 0){
		if(( temperaturSolar >= temperaturBufferMean + hysteresisON) && (temperaturSolar >= temperaturSolarMin) && (temperaturBufferMean <= temperaturBufferMax)){
			relaisState = 1;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);
		} else {
			relaisState = 0;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
	}
}
	if(relaisState == 1){
		if ((temperaturBufferMean >= temperaturSolar + hysteresisOFF) || (temperaturSolar < temperaturSolarMin) || (temperaturBufferMean > temperaturBufferMax)){
			relaisState = 0;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
		} else {
			relaisState = 1;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);		
		}
	}
}else {
		if(sens3Solar && !sens3Buffer){
	if(relaisState == 0){
		if(( temperaturSolarMean >= temperaturBuffer + hysteresisON) && (temperaturSolarMean >= temperaturSolarMin) && (temperaturBuffer <= temperaturBufferMax)){
			relaisState = 1;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);
		} else {
			relaisState = 0;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
	}
}
	if(relaisState == 1){
		if ((temperaturBuffer >= temperaturSolarMean + hysteresisOFF) || (temperaturSolarMean < temperaturSolarMin) || (temperaturBuffer > temperaturBufferMax)){
			relaisState = 0;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
		} else {
			relaisState = 1;
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);		
		}
	}			
		}
	}
}
} else {
	relaisState = 0;
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
}
	
	if(relaisStateOld == 0 && relaisState == 1){
		tmpTime.Seconds = 0;
		tmpTime.Minutes = 0;
		tmpTime.Hours = 0;
		HAL_RTC_SetTime(&hrtc, &tmpTime, RTC_FORMAT_BCD);
	}
	if(relaisState == 0 && relaisStateOld == 1){
		HAL_RTC_GetTime(&hrtc, &tmpTime, RTC_FORMAT_BCD);
		onTime += tmpTime.Seconds + tmpTime.Minutes * 60 + tmpTime.Hours * 3600; 
	}
	


  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles SPI2 global interrupt.
*/
void SPI2_IRQHandler(void)
{
  /* USER CODE BEGIN SPI2_IRQn 0 */

  /* USER CODE END SPI2_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi2);
  /* USER CODE BEGIN SPI2_IRQn 1 */

  /* USER CODE END SPI2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
