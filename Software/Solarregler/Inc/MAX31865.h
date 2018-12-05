#ifndef MAX31865_H
#define MAX31865_H

#include "stm32f1xx_hal.h"

#define CONFIG_REGISTER_READ 0x00
#define CONFIG_REGISTER_WRITE 0x80
#define RTD_MSB_REGISTER_READ 0x01
#define RTD_LSB_REGISTER_READ 0x02
#define FAULT_STATUS_REGISTER_READ 0x07

#define ONE_SHOT_CONFIG ((0x1 << 7 )|(0x1 << 5)|(0x1 << 0))

typedef struct{
	
	GPIO_TypeDef * CS_GPIOx;					// GPIO Port for Chip Select
	uint16_t CS_Pin;									// GPIO Pin for Chhip Select
	SPI_HandleTypeDef * SPI_Handle;		// SPI Handle for Communication
	GPIO_TypeDef * RDY_GPIOx;					// GPIO Port for READY Pin
	uint16_t RDY_Pin;									// GPIO Pin for READY Pin
	double referenceResistor;					// Value of reference resistor
	
} MAX31865_TypeDef;




double measureTemperatureOneShotConverted(MAX31865_TypeDef * MAX31865);
uint16_t measureTemperatureOneShotAdcRaw(MAX31865_TypeDef * MAX31865);

void singleByteWrite(uint8_t address, uint8_t data, SPI_HandleTypeDef* SPI, GPIO_TypeDef *CS_GPIOx, uint16_t CS_Pin);
uint8_t singleByteRead(uint8_t address, SPI_HandleTypeDef* SPI, GPIO_TypeDef *CS_GPIOx, uint16_t CS_Pin);






#endif