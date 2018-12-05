
#include "MAX31865.h"
#include "stm32f1xx_hal.h"


///////////////////////////////////////////////////////////////////////////////////////
// MAX31865: Handle Struct for MAX31865 Chip


double measureTemperatureOneShotConverted(MAX31865_TypeDef * MAX31865){  //measures temperature and gives back value in °C
	double temperature;
	double percentage;
	double resistance;
	
	percentage = (double)measureTemperatureOneShotAdcRaw(MAX31865) / (double)((1 << 15) - 1);
	
	resistance = (percentage * MAX31865->referenceResistor);
	
	temperature = (resistance - 100.0) * 2.597;
	
	return temperature;
}



///////////////////////////////////////////////////////////////////////////////////////
// MAX31865: Handle Struct for MAX31865 Chip

uint16_t measureTemperatureOneShotAdcRaw(MAX31865_TypeDef * MAX31865){
	uint16_t adcCount;
	
	singleByteWrite(CONFIG_REGISTER_WRITE, ONE_SHOT_CONFIG, MAX31865->SPI_Handle, MAX31865->CS_GPIOx, MAX31865->CS_Pin);
	
	while(HAL_GPIO_ReadPin(MAX31865->RDY_GPIOx, MAX31865->RDY_Pin) == 1);
	
	adcCount = (singleByteRead(RTD_MSB_REGISTER_READ, MAX31865->SPI_Handle, MAX31865->CS_GPIOx, MAX31865->CS_Pin) << 8);
	
	adcCount |= singleByteRead(RTD_LSB_REGISTER_READ, MAX31865->SPI_Handle, MAX31865->CS_GPIOx, MAX31865->CS_Pin);
	
	adcCount >>= 1;
	
	
	return adcCount;
}

///////////////////////////////////////////////////////////////////////////////////////
// address: adress of register to write
// data: data to write to the register
// SPI: SPI Handle to send data
// CS_GPIOx: GPIO Handle for port of CS pin
// CS_Pin: Number of CS pin

void singleByteWrite(uint8_t address, uint8_t data, SPI_HandleTypeDef* SPI, GPIO_TypeDef *CS_GPIOx, uint16_t CS_Pin){
	
	HAL_GPIO_WritePin(CS_GPIOx, CS_Pin, GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(SPI, &address, sizeof(address), HAL_MAX_DELAY);
	
	HAL_SPI_Transmit(SPI, &data, sizeof(data), HAL_MAX_DELAY);
	
	HAL_GPIO_WritePin(CS_GPIOx, CS_Pin, GPIO_PIN_SET);
	
}


///////////////////////////////////////////////////////////////////////////////////////
// address: adress of register to write
// SPI: SPI Handle to send data
// CS_GPIOx: GPIO Handle for port of CS pin
// CS_Pin: Number of CS pin

uint8_t singleByteRead(uint8_t address, SPI_HandleTypeDef* SPI, GPIO_TypeDef *CS_GPIOx, uint16_t CS_Pin){
	
	uint8_t sendData[2];
	uint8_t receiveData[2];
	
	sendData[0] = address;
	sendData[1] = 0x00; // Dummy
	
	HAL_GPIO_WritePin(CS_GPIOx, CS_Pin, GPIO_PIN_RESET);
	
	HAL_SPI_TransmitReceive(SPI, sendData, receiveData, sizeof(receiveData), HAL_MAX_DELAY);
	
	HAL_GPIO_WritePin(CS_GPIOx, CS_Pin, GPIO_PIN_SET);
	
	return receiveData[1];
	
}




