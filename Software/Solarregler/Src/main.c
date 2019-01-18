
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#include "MAX31865.h"
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

float temperaturSolar = 0.0;
float temperaturSens3 = 0.0;
float temperaturSolarMean = 0.0;
float temperaturBuffer = 0.0;
float temperaturBufferMean = 0.0;

float temperaturSolarMin = 50;
float temperaturBufferMax = 90;

float hysteresisON = 10.0;
float hysteresisOFF = 2.0;

float resistanceSolar = 0.0;
float resistanceBuffer  = 0.0;

MAX31865_TypeDef MAXSolar;
MAX31865_TypeDef MAXBuffer;
MAX31865_TypeDef MAXSens3;

uint8_t relaisState = 0;

uint8_t controllerActivated = 0;

uint8_t sens3Used = 0;

uint8_t sens3Solar = 0;

uint8_t sens3Buffer = 0;

float onTime = 0;

uint16_t CanBaseAdress = 0x120;


CanRxMsgTypeDef CanRx;
CanTxMsgTypeDef CanTx;
CAN_FilterConfTypeDef filterConfig;


	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint8_t readCanIdDip(void);

void serialize(float data, uint8_t * serial);

void reenableCanInterrupt(CAN_HandleTypeDef* hcan);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	
	// Initialization of the MAX PT100 Chips
	MAXSolar.SPI_Handle = &hspi2;
	MAXSolar.CS_Pin = CS_1_Pin;
	MAXSolar.CS_GPIOx = CS_1_GPIO_Port;
	MAXSolar.RDY_Pin = DRDY_1_Pin;
	MAXSolar.RDY_GPIOx = DRDY_1_GPIO_Port;
	MAXSolar.referenceResistor = 430.0;
	
	MAXSens3.SPI_Handle = &hspi2;
	MAXSens3.CS_Pin = CS_3_Pin;
	MAXSens3.CS_GPIOx = CS_3_GPIO_Port;
	MAXSens3.RDY_Pin = DRDY_3_Pin;
	MAXSens3.RDY_GPIOx = DRDY_3_GPIO_Port;
	MAXSens3.referenceResistor = 430.0;
	
	MAXBuffer.SPI_Handle = &hspi2;
	MAXBuffer.CS_Pin = CS_2_Pin;
	MAXBuffer.CS_GPIOx = CS_2_GPIO_Port;
	MAXBuffer.RDY_Pin = DRDY_2_Pin;
	MAXBuffer.RDY_GPIOx = DRDY_2_GPIO_Port;
	MAXBuffer.referenceResistor = 430.0;
	
	initialize(&MAXSolar, STANDARD_INIT);
	//initialize(&MAXSens3, STANDARD_INIT);
	initialize(&MAXBuffer, STANDARD_INIT);

	
	checkForFaults(&MAXSolar);
	//checkForFaults(&MAXSens3);
	checkForFaults(&MAXBuffer);
	
	/////////////////////////////////////////////////
	
	//CanBaseAdress = readCanIdDip();
	
	// Initialization of CAN Messages and Filter
	hcan.pTxMsg = &CanTx;
	hcan.pTxMsg->StdId = CanBaseAdress + 0x1;
	hcan.pTxMsg->ExtId = 0x0;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;
	hcan.pTxMsg->IDE = CAN_ID_STD;
	hcan.pTxMsg->DLC = 5; //Datenmenge max 8
	
	hcan.pRxMsg = &CanRx;
	hcan.pRxMsg->IDE = CAN_ID_STD;
	hcan.pRxMsg->FIFONumber = CAN_FIFO0;
	hcan.pRxMsg->FMI = 1;
	
	filterConfig.FilterNumber = 1;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterIdLow = (CanBaseAdress | 0x00F) << 5; // wegen Fehler? offset der ID um 5 Bits
	filterConfig.FilterIdHigh = (CanBaseAdress + 0x7) << 5;
	filterConfig.FilterMaskIdLow = (CanBaseAdress + 0xB) << 5;
	filterConfig.FilterMaskIdHigh = 0x0;
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filterConfig.FilterActivation = ENABLE;
	filterConfig.BankNumber = 14;
	/////////////////////////////////////////////////
	
	
	// Setup Filter
	HAL_CAN_ConfigFilter(&hcan, &filterConfig);
	
	//Start Interrupt
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	
	
	// Timer start
	HAL_TIM_Base_Start_IT(&htim2);
	
	

	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
	
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_9TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4096;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PB5   ------> I2C1_SMBA
     PB6   ------> I2C1_SCL
     PB7   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LD1_Pin|CS_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_2_GPIO_Port, CS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_3_Pin|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD1_Pin CS_1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD1_Pin|CS_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_1_Pin */
  GPIO_InitStruct.Pin = DRDY_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DRDY_2_Pin */
  GPIO_InitStruct.Pin = DRDY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRDY_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_2_Pin */
  GPIO_InitStruct.Pin = CS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRDY_3_Pin PB11 */
  GPIO_InitStruct.Pin = DRDY_3_Pin|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_3_Pin PB12 */
  GPIO_InitStruct.Pin = CS_3_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : C7_Pin C6_Pin C5_Pin */
  GPIO_InitStruct.Pin = C7_Pin|C6_Pin|C5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : C4_Pin C3_Pin C2_Pin C1_Pin 
                           C0_Pin */
  GPIO_InitStruct.Pin = C4_Pin|C3_Pin|C2_Pin|C1_Pin 
                          |C0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint8_t readCanIdDip(){
	
	uint8_t canIdDip;
	
	// read in the individual DIP ID Pins
	canIdDip = (HAL_GPIO_ReadPin(C0_GPIO_Port, C0_Pin) << 0) | (HAL_GPIO_ReadPin(C1_GPIO_Port, C1_Pin) << 1) | (HAL_GPIO_ReadPin(C2_GPIO_Port, C2_Pin) << 2) | (HAL_GPIO_ReadPin(C3_GPIO_Port, C3_Pin) << 3);
	canIdDip |= (HAL_GPIO_ReadPin(C4_GPIO_Port, C4_Pin) << 4) | (HAL_GPIO_ReadPin(C5_GPIO_Port, C5_Pin) << 5) | (HAL_GPIO_ReadPin(C6_GPIO_Port, C6_Pin) << 6) | (HAL_GPIO_ReadPin(C7_GPIO_Port, C7_Pin) << 7);
	
	return canIdDip;
}


void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	
	
if(hcan->pRxMsg->StdId == (CanBaseAdress + MESSAGE_TRIGGER_OFFSET)){
	
		if(sens3Used == 0){
			
		hcan->pTxMsg->StdId = (CanBaseAdress + SENS_1_OFFSET);
		hcan->pTxMsg->DLC = 5;
		hcan->pTxMsg->Data[0] = MAXSolar.faultDetected;
		serialize(temperaturSolar, &(hcan->pTxMsg->Data[1]));
		HAL_CAN_Transmit(hcan, 10);
		
		} else {
		
		if(sens3Solar && !sens3Buffer){
			
		hcan->pTxMsg->StdId = (CanBaseAdress + SENS_1_OFFSET);
		hcan->pTxMsg->DLC = 5;
		hcan->pTxMsg->Data[0] = MAXSolar.faultDetected | MAXSens3.faultDetected;
		serialize(temperaturSolarMean, &(hcan->pTxMsg->Data[1]));
		HAL_CAN_Transmit(hcan, 10);		
					
			}	else {
				
		if(sens3Buffer && !sens3Solar){
			
		hcan->pTxMsg->StdId = (CanBaseAdress + SENS_1_OFFSET);
		hcan->pTxMsg->DLC = 5;
		hcan->pTxMsg->Data[0] = MAXBuffer.faultDetected | MAXSens3.faultDetected;
		serialize(temperaturBufferMean, &(hcan->pTxMsg->Data[1]));
		HAL_CAN_Transmit(hcan, 10);	
			
				}
			}		
		}

		hcan->pTxMsg->StdId = (CanBaseAdress + SENS_2_OFFSET);
		hcan->pTxMsg->DLC = 5;
		hcan->pTxMsg->Data[0] = MAXBuffer.faultDetected ;
		serialize(temperaturBuffer, &(hcan->pTxMsg->Data[1]));
		HAL_CAN_Transmit(hcan, 10);
	
		hcan->pTxMsg->StdId = (CanBaseAdress + SCHWELLE_AN_AUS_OFFSET);
		hcan->pTxMsg->DLC = 8;
		serialize(hysteresisON, &(hcan->pTxMsg->Data[0]));
		serialize(hysteresisOFF, &(hcan->pTxMsg->Data[4]));
		HAL_CAN_Transmit(hcan, 10);
	
		hcan->pTxMsg->StdId = (CanBaseAdress + INFO_RELAIS_OFFSET);
		hcan->pTxMsg->DLC = 1;
	  hcan->pTxMsg->Data[0] = relaisState;
		HAL_CAN_Transmit(hcan, 10);
		
		hcan->pTxMsg->StdId = (CanBaseAdress + TEMPERATUR_MIN_MAX_OFFSET);
		hcan->pTxMsg->DLC = 8;
		serialize(temperaturBufferMax, &(hcan->pTxMsg->Data[0]));
		serialize(temperaturSolarMin, &(hcan->pTxMsg->Data[4]));
		HAL_CAN_Transmit(hcan, 10);
			
		hcan->pTxMsg->StdId = (CanBaseAdress + REGLER_AKTIV_OFFSET);
		hcan->pTxMsg->DLC = 1;
	  hcan->pTxMsg->Data[0] = *((uint8_t *) &controllerActivated);
		HAL_CAN_Transmit(hcan, 10);
		
		hcan->pTxMsg->StdId = (CanBaseAdress + KORREKTUR_WIDERSTAND_OFFSET);
		hcan->pTxMsg->DLC = 8;
		serialize(resistanceSolar, &(hcan->pTxMsg->Data[0]));
		serialize(resistanceBuffer, &(hcan->pTxMsg->Data[4]));
		HAL_CAN_Transmit(hcan, 10);
		
		hcan->pTxMsg->StdId = (CanBaseAdress + ON_TIME_OFFSET);
		hcan->pTxMsg->DLC = 4;
		serialize(onTime, &(hcan->pTxMsg->Data[0]));
		HAL_CAN_Transmit(hcan, 10);

}	


if(hcan->pRxMsg->StdId == (CanBaseAdress + SET_SCHWELLE_AN_AUS_OFFSET)){
	float tmpON;
	float tmpOFF;
	uint8_t * ON = (uint8_t *) &tmpON;
	uint8_t * OFF = (uint8_t *) &tmpOFF;

	
	*ON = hcan->pRxMsg->Data[0];
	*(ON + 1) = hcan->pRxMsg->Data[1];
	*(ON + 2) = hcan->pRxMsg->Data[2];
	*(ON + 3) = hcan->pRxMsg->Data[3];

	*OFF = hcan->pRxMsg->Data[4];
	*(OFF + 1) = hcan->pRxMsg->Data[5];
	*(OFF + 2) = hcan->pRxMsg->Data[6];
	*(OFF + 3) = hcan->pRxMsg->Data[7];
	
	if(tmpON <= MAX_ADJUSTABLE_TEMPERATURE && tmpON >= MIN_ADJUSTABLE_TEMPERATURE){
		hysteresisON = tmpON;
	} else {
		if(tmpON > MAX_ADJUSTABLE_TEMPERATURE){
			hysteresisON = MAX_ADJUSTABLE_TEMPERATURE;
		}
		if(tmpON < MIN_ADJUSTABLE_TEMPERATURE){
			hysteresisON = MIN_ADJUSTABLE_TEMPERATURE;
		}
	}

	
	if(tmpOFF <= MAX_ADJUSTABLE_TEMPERATURE && tmpOFF >= MIN_ADJUSTABLE_TEMPERATURE){
		hysteresisOFF = tmpOFF;
	} else {
		if(tmpOFF > MAX_ADJUSTABLE_TEMPERATURE){
			hysteresisOFF = MAX_ADJUSTABLE_TEMPERATURE;
		}
		if(tmpOFF < MIN_ADJUSTABLE_TEMPERATURE){
			hysteresisOFF = MIN_ADJUSTABLE_TEMPERATURE;
		}
	}
	
	
		hcan->pTxMsg->StdId = (CanBaseAdress + SCHWELLE_AN_AUS_OFFSET);
		hcan->pTxMsg->DLC = 8;
		serialize(hysteresisON, &(hcan->pTxMsg->Data[0]));
		serialize(hysteresisOFF, &(hcan->pTxMsg->Data[4]));
		HAL_CAN_Transmit(hcan, 10);
	
}
	
if(hcan->pRxMsg->StdId == (CanBaseAdress + SET_TEMPERATUR_MIN_MAX_OFFSET)){
	float tmpMAX;
	float tmpMIN;
	uint8_t * MAX = (uint8_t *) &tmpMAX;
	uint8_t * MIN = (uint8_t *) &tmpMIN;
	
	*MAX = hcan->pRxMsg->Data[0];
	*(MAX + 1) = hcan->pRxMsg->Data[1];
	*(MAX + 2) = hcan->pRxMsg->Data[2];
	*(MAX + 3) = hcan->pRxMsg->Data[3];

	*MIN = hcan->pRxMsg->Data[4];
	*(MIN + 1) = hcan->pRxMsg->Data[5];
	*(MIN + 2) = hcan->pRxMsg->Data[6];
	*(MIN + 3) = hcan->pRxMsg->Data[7];
	
	
		if(tmpMAX <= MAX_ADJUSTABLE_TEMPERATURE && tmpMAX >= MIN_ADJUSTABLE_TEMPERATURE){
		temperaturBufferMax = tmpMAX;
		} else {
		if(tmpMAX > MAX_ADJUSTABLE_TEMPERATURE){
			temperaturBufferMax = MAX_ADJUSTABLE_TEMPERATURE;
		}
		if(tmpMAX < MIN_ADJUSTABLE_TEMPERATURE){
			temperaturBufferMax = MIN_ADJUSTABLE_TEMPERATURE;
		}
	}
		
		if(tmpMIN <= MAX_ADJUSTABLE_TEMPERATURE && tmpMIN >= MIN_ADJUSTABLE_TEMPERATURE){
		temperaturSolarMin = tmpMIN;
		} else {
		if(tmpMIN > MAX_ADJUSTABLE_TEMPERATURE){
			temperaturSolarMin = MAX_ADJUSTABLE_TEMPERATURE;
		}
		if(tmpMIN < MIN_ADJUSTABLE_TEMPERATURE){
			temperaturSolarMin = MIN_ADJUSTABLE_TEMPERATURE;
		}
	}
	
	
		hcan->pTxMsg->StdId = (CanBaseAdress + TEMPERATUR_MIN_MAX_OFFSET);
		hcan->pTxMsg->DLC = 8;
		serialize(temperaturBufferMax, &(hcan->pTxMsg->Data[0]));
		serialize(temperaturSolarMin, &(hcan->pTxMsg->Data[4]));
		HAL_CAN_Transmit(hcan, 10);
	
}

if(hcan->pRxMsg->StdId == (CanBaseAdress + SET_REGLER_AKTIV_OFFSET)){
	
	controllerActivated = hcan->pRxMsg->Data[0];
	
		hcan->pTxMsg->StdId = (CanBaseAdress + 0x6);
		hcan->pTxMsg->DLC = 1;
	  hcan->pTxMsg->Data[0] = *((uint8_t *) &controllerActivated);
		HAL_CAN_Transmit(hcan, 10);
	
}
	

if(hcan->pRxMsg->StdId == (CanBaseAdress + SET_KORREKTUR_WIDERSTAND_OFFSET)){
	

	uint8_t * SOL = (uint8_t *) &resistanceSolar;
	uint8_t * BUF = (uint8_t *) &resistanceBuffer;
	
	*SOL = hcan->pRxMsg->Data[0];
	*(SOL + 1) = hcan->pRxMsg->Data[1];
	*(SOL + 2) = hcan->pRxMsg->Data[2];
	*(SOL + 3) = hcan->pRxMsg->Data[3];

	*BUF = hcan->pRxMsg->Data[4];
	*(BUF + 1) = hcan->pRxMsg->Data[5];
	*(BUF + 2) = hcan->pRxMsg->Data[6];
	*(BUF + 3) = hcan->pRxMsg->Data[7];
	
	
	hcan->pTxMsg->StdId = (CanBaseAdress + KORREKTUR_WIDERSTAND_OFFSET);
	hcan->pTxMsg->DLC = 8;
	serialize(resistanceSolar, &(hcan->pTxMsg->Data[0]));
	serialize(resistanceBuffer, &(hcan->pTxMsg->Data[4]));
	HAL_CAN_Transmit(hcan, 10);
	
}

if(hcan->pRxMsg->StdId == (CanBaseAdress + SET_SENS_3_STATUS_OFFSET)){
	
	sens3Used = hcan->pRxMsg->Data[0];
	sens3Buffer = hcan->pRxMsg->Data[1];
	sens3Solar = hcan->pRxMsg->Data[2];
	
}
	
	reenableCanInterrupt(hcan);
	
}


void reenableCanInterrupt(CAN_HandleTypeDef* hcan){
	
		if(hcan->State == HAL_CAN_STATE_BUSY_TX){
			hcan->State = HAL_CAN_STATE_BUSY_TX_RX0; 
		} else {
			hcan->State = HAL_CAN_STATE_BUSY_RX0;
			hcan->ErrorCode = HAL_CAN_ERROR_NONE;
		
			__HAL_CAN_ENABLE_IT(hcan, CAN_IT_EWG);		
			__HAL_CAN_ENABLE_IT(hcan, CAN_IT_EPV);
			__HAL_CAN_ENABLE_IT(hcan, CAN_IT_BOF);
			__HAL_CAN_ENABLE_IT(hcan, CAN_IT_LEC);
			__HAL_CAN_ENABLE_IT(hcan, CAN_IT_ERR);
			__HAL_CAN_ENABLE_IT(hcan, CAN_IT_EWG);
		
	}
	
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
	
}


void serialize(float data, uint8_t * serial){
	
		*serial = *((uint8_t *) &data);
		*(serial + 1) = *(((uint8_t *) &data) + 1);
		*(serial + 2) = *(((uint8_t *) &data) + 2);
		*(serial + 3) = *(((uint8_t *) &data) + 3);
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
