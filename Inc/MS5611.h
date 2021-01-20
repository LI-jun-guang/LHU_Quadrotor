	/*
	*@file : MS5611_test
	*@author : Guang 
	*@date : 2018/04/17
	* 
	*
	*
	*
	*/
	
/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define CS_MS_L HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET)
#define CS_MS_H HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET)

#define MS5611_CMD_Reset            	0x1E
#define MS5611_CMD_AD_Read          	0x00

#define MS5611_CMD_PROM_Read_Setup  	0xA0
#define MS5611_CMD_PROM_Read_C1     	0xA2
#define MS5611_CMD_PROM_Read_C2     	0xA4
#define MS5611_CMD_PROM_Read_C3     	0xA6
#define MS5611_CMD_PROM_Read_C4     	0xA8
#define MS5611_CMD_PROM_Read_C5     	0xAA
#define MS5611_CMD_PROM_Read_C6    		0xAC
#define MS5611_CMD_PROM_Read_CCR    	0xAE

#define MS5611_D1_CONV_OSR_256  			0x40
#define MS5611_D1_CONV_OSR_512  			0x42
#define MS5611_D1_CONV_OSR_1024 			0x44
#define MS5611_D1_CONV_OSR_2048 			0x46
#define MS5611_D1_CONV_OSR_4096 			0x48

#define MS5611_D2_CONV_OSR_256  			0x50
#define MS5611_D2_CONV_OSR_512  			0x52
#define MS5611_D2_CONV_OSR_1024 			0x54
#define MS5611_D2_CONV_OSR_2048 			0x56
#define MS5611_D2_CONV_OSR_4096 			0x58

typedef union
{
	uint8_t Byte[3];
	uint32_t data;
}raw;

typedef struct
{
	uint16_t C[7];
	uint32_t D1,D2;
	int32_t Dt;
	int32_t TEMP;
	int64_t OFF;
	int64_t OFF2;
	int64_t SENS;
	int64_t SENS2;
	int32_t P;
	int32_t T2;
}Data;


void MS5611_Pressure(void);
void Cal_temp(void);
float Cal_Temp_Comp_Pressure(void);
 
void MS5611_init(void);
void MS5611_Write( uint8_t address );
uint16_t MS5611_Read_16Bit(uint8_t address);
uint32_t MS5611_Read_24Bit(uint8_t address);












