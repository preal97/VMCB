#ifndef MAX31865_H
#define MAX31865_H

#include "stm32f1xx_hal.h"

#define CONFIG_REGISTER_READ 0x00
#define RTD_MSB_REGISTER_READ 0x01
#define RTD_LSB_REGISTER_READ 0x02
#define FAULT_STATUS_REGISTER_READ 0x07
#define HIGH_FAULT_THRESHOLD_MSB_READ 0x03
#define HIGH_FAULT_THRESHOLD_LSB_READ 0x04
#define LOW_FAULT_THRESHOLD_MSB_READ 0x05
#define LOW_FAULT_THRESHOLD_LSB_READ 0x06

#define CONFIG_REGISTER_WRITE 0x80
#define HIGH_FAULT_THRESHOLD_MSB_WRITE 0x83
#define HIGH_FAULT_THRESHOLD_LSB_WRITE 0x84
#define LOW_FAULT_THRESHOLD_MSB_WRITE 0x85
#define LOW_FAULT_THRESHOLD_LSB_WRITE 0x86


//#define ONE_SHOT_CONFIG ((0x1 << 7 )|(0x1 << 5)|(0x1 << 0))
#define VBIAS_ON (0x01 << 7)
#define CONVERSION_AUTO_ON (0x01 << 6)
#define ONE_SHOT (0x01 << 5)
#define THREEWIRED_ON (0x01 << 4)
#define FAULT_STATUS_CLEAR (0x01 << 1)
#define FREQ_50HZ (0x01 << 0)

#define STANDARD_INIT (VBIAS_ON | FREQ_50HZ | FAULT_STATUS_CLEAR)

typedef struct{
	
	GPIO_TypeDef * CS_GPIOx;					// GPIO Port for Chip Select
	uint16_t CS_Pin;									// GPIO Pin for Chhip Select
	SPI_HandleTypeDef * SPI_Handle;		// SPI Handle for Communication
	GPIO_TypeDef * RDY_GPIOx;					// GPIO Port for READY Pin
	uint16_t RDY_Pin;									// GPIO Pin for READY Pin
	double referenceResistor;					// Value of reference resistor
	uint8_t faultDetected;						// is set to value unequal 0 if fault was detected using checkForFaults
	
} MAX31865_TypeDef;


void initialize(MAX31865_TypeDef * MAX31865, uint8_t setupCodes);
void checkForFaults(MAX31865_TypeDef * MAX31865);

double measureTemperatureOneShotConverted(MAX31865_TypeDef * MAX31865);
uint16_t measureTemperatureOneShotAdcRaw(MAX31865_TypeDef * MAX31865);

void singleByteWrite(uint8_t address, uint8_t data, SPI_HandleTypeDef* SPI, GPIO_TypeDef *CS_GPIOx, uint16_t CS_Pin);
uint8_t singleByteRead(uint8_t address, SPI_HandleTypeDef* SPI, GPIO_TypeDef *CS_GPIOx, uint16_t CS_Pin);






#endif