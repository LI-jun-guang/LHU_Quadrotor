	/*
	*@file : MS5611_test
	*@author : Guang 
	*@date : 2018/04/17
	*@drief : 
	*
	*
	*--------------------------------------------------------------*
	*SPI Receiver                                                  *
	*1.Brinig slave select low									      						 *
	*2.Transmit register + 0x80(to set MSB bit to high,Read mode)  *
	*3.Receive data												   											 *
	*4.Brinig slave select high                                    *
	*--------------------------------------------------------------*
	*SPI Transmit                                                  *
	*1.Brinig slave select low									   								 *
	*2.Transmit register + data                                    *
	*3.Brinig slave select high                                    *
	*/                                                        
/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "MS5611.h"
#include "Math.h"

extern SPI_HandleTypeDef hspi3;
raw SPI_MS_RxBuff;
Data MS5611;
//float MS5611_Lowpass_lastdata=0,MS5611_Lowpass_output;



void MS5611_init(void)
{	
	MS5611.C[1] = MS5611_Read_16Bit(MS5611_CMD_PROM_Read_C1);
	MS5611.C[2] = MS5611_Read_16Bit(MS5611_CMD_PROM_Read_C2);
	MS5611.C[3] = MS5611_Read_16Bit(MS5611_CMD_PROM_Read_C3);
	MS5611.C[4] = MS5611_Read_16Bit(MS5611_CMD_PROM_Read_C4);
	MS5611.C[5] = MS5611_Read_16Bit(MS5611_CMD_PROM_Read_C5);
	MS5611.C[6] = MS5611_Read_16Bit(MS5611_CMD_PROM_Read_C6);
}

//float Cal_Temp_Comp_Pressure(void)
//{
//	
//	//Read D1
//	MS5611_Write(MS5611_D1_CONV_OSR_256);
//	HAL_Delay(1);
//	MS5611.D1 = MS5611_Read_24Bit(MS5611_CMD_AD_Read);
//	//Read D2
//	MS5611_Write(MS5611_D2_CONV_OSR_256);
//	HAL_Delay(1);
//	MS5611.D2 = MS5611_Read_24Bit(MS5611_CMD_AD_Read);
//	//Read Dt and TEMP
//	MS5611.Dt = MS5611.D2 - MS5611.C[5]*256;
//	MS5611.TEMP = 2000+((MS5611.Dt/100)*(MS5611.C[6]/100)/83886.08)*100;
//  
//	//MS5611溫度補償 
//	if(MS5611.TEMP >=2000) 
//	{
//		MS5611.OFF2 = 0;
//	  MS5611.SENS2 = 0;
//	}
//	else if (MS5611.TEMP < 2000) 
//	{
//		MS5611.T2 = (MS5611.Dt*MS5611.Dt)/pow(2,31);
//		MS5611.OFF2 = 5*(MS5611.TEMP-pow(2000,2))/2;
//	  MS5611.SENS2 = 5*pow((MS5611.TEMP-2000),2)/4;
//	}
//	
//	//更新溫度資料
//	MS5611.TEMP = MS5611.TEMP -MS5611.T2;
//	MS5611.OFF = (((MS5611.C[2]/100)*655.35)*10000+(((MS5611.C[4]/100)*(MS5611.Dt/100))/1.28)*100)-MS5611.OFF2;
//	MS5611.SENS = (((MS5611.C[1]/100)*327.68)*10000 +((MS5611.C[3]/100)*(MS5611.Dt/100)/2.56)*100)-MS5611.SENS2;
//	MS5611.P = ((MS5611.D1 * MS5611.SENS/2097152)-MS5611.OFF)/32768;
//	MS5611_Lowpass_output = (MS5611_Lowpass_lastdata*0.99) + (MS5611.P*0.01);
//	MS5611_Lowpass_lastdata = MS5611_Lowpass_output;
//	
//	
//	return MS5611_Lowpass_output;
//}
/* MS5611_Read & Write ---------------------------------------------------------------------*/

//MS5611 SPI Receiver for 16bit data 
uint16_t MS5611_Read_16Bit(uint8_t address)
{
	uint8_t spi_Buff;
	uint16_t return_value; 
	
	spi_Buff = (address|0x80);
	CS_MS_L;
	HAL_SPI_Transmit(&hspi3, &spi_Buff,1,10);
	
	HAL_SPI_Receive(&hspi3, SPI_MS_RxBuff.Byte,2,10);
	CS_MS_H;
	
	return_value = SPI_MS_RxBuff.Byte[0]<<8 | SPI_MS_RxBuff.Byte[1];

	return return_value;
}
	

//MS5611 SPI Receiver for 32bit data 
uint32_t MS5611_Read_24Bit(uint8_t address)
{
	uint8_t spi_Buff;
	uint32_t return_value; 

	spi_Buff = (address);
	CS_MS_L;
	HAL_SPI_Transmit(&hspi3, &spi_Buff,1,10);
	HAL_SPI_Receive(&hspi3,SPI_MS_RxBuff.Byte,3,10);
	CS_MS_H;
	
	return_value = SPI_MS_RxBuff.Byte[0]<<16 | SPI_MS_RxBuff.Byte[1]<<8 | SPI_MS_RxBuff.Byte[2];
	
	return return_value;
}

//MS5611 SPI Write  
void MS5611_Write(uint8_t address)
{
		CS_MS_L;
		HAL_SPI_Transmit(&hspi3,&address,1,10);
		CS_MS_H;
}



