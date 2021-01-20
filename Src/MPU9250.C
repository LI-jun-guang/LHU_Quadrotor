/*
	*@file : MPU9250
	*@author : Guang 
	*@date : 2018/08/11
	*
	*
	*--------------------------------------------------------------*
	*SPI Receiver                                                  *
	*1.Brinig slave select low																		 *
	*2.Transmit register + 0x80(to set MSB bit to high,Read mode)  *
	*3.Receive data																								 *
	*4.Brinig slave select high                                    *
	*--------------------------------------------------------------*
	*SPI Transmit                                                  *
	*1.Brinig slave select low																		 *
	*2.Transmit register + data                                    *
	*3.Brinig slave select high                                    *
	*/                                                             
/* Includes -------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "MPU9250.h"
#include "Math.h"
/*-----------------------------------------------------------------*/
uint8_t SPI_TxBuff[2];
MPU9250_Raw SPI_MP_RxBuff;
float MAG_ASA[3]; //AK8963 asax,asay,asaz data

void SPI_Configuration(uint32_t BaudRatePrescaler);

extern SPI_HandleTypeDef hspi3;

/*-----------------------------------------------------------------*/

void MPU9250_Configuration(){

	MPU9250_Write(MPU9250_PWR_MGMT_1,0x80); 												//Resrt device , Sleep mode disable, use internal 20MHz oscillator
	HAL_Delay(100);
	
	MPU9250_Write(MPU9250_SIGNAL_PATH_RESET,0x07); 									//Resrt digital signal path
	HAL_Delay(100);
	
	MPU9250_Write(MPU9250_PWR_MGMT_2,0x00);
	MPU9250_Write(MPU9250_USER_CTRL,0x1F); 													//Resrt  DMP , FIFO , I2C_MST , SIG_COND Set SPI interface only
	while(MPU9250_Read(MPU9250_WHO_AM_I)!=0x0071){									//Return 0x71
	printf("MPU9250 communication fail\r\n");} 
	
	MPU9250_Write(MPU9250_FIFO_Enable,0x00);  											//Disable FIFO 
	MPU9250_Write(MPU9250_CONFIG,0x01);															//Set Gyro 250hz bandwidth, 8khz Fs
	MPU9250_Write(MPU9250_GYRO_CONFIG,MPU9250_GYRO_FS_250 ); 				//Set FCHOICE_B=0, Gyro Full Scale +-250dps , Gyro self-test off
	MPU9250_Write(MPU9250_ACCEL_CONFIG_2,0x06); 										//Set ACC	99Hz bandwidth, 1khz Fs
	MPU9250_Write(MPU9250_ACCEL_CONFIG,MPU9250_ACCEL_FS_2);  				//Set Accel Full Scale +-16g , Accel self-test off
			
	MPU9250_Write(MPU9250_USER_CTRL,0X20); 													//Enable I2C Master module
	MPU9250_Write(MPU9250_I2C_MST_CTRL,0X0D);												//I2C 400KHz
	
	MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963|0X80);  	// Set the I2C slave address of AK8963 and set read.
	MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_WAI);
	MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0X81);											// Enable I2C and transfer 1 byte
	HAL_Delay(50);
	
	while(	MPU9250_Read(EXT_SENS_DATA_00)!=0x0048){								//Return 0x48
	printf("MPU9250 AK8963 SLAVE communication fail\r\n");} 


	MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963);
	MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_CNTL2); 
	MPU9250_Write(MPU9250_I2C_SLV0_DO,0X01); 												//Reset device 
	MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0X81);
	HAL_Delay(50);
	
	MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963); 
	MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_CNTL1); 
	MPU9250_Write(MPU9250_I2C_SLV0_DO,0X00);												//Power down magnetometer 
	MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0X81);
	HAL_Delay(50);
	
	MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963); 
	MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_CNTL1); 
	MPU9250_Write(MPU9250_I2C_SLV0_DO,0X0F);												//Enter Fuse ROM access mode
	MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0X81);
	HAL_Delay(50);
	
	MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963|0X80);
	MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_ASAX);
	MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0X83);          						//Enable I2C and read 3 bytes
	HAL_Delay(50);

  MAG_ASA[0] = (((MPU9250_Read(EXT_SENS_DATA_00)-128)*0.5)/128)+1;//Read the x-, y-, and z-axis calibration values
  MAG_ASA[1] = (((MPU9250_Read(EXT_SENS_DATA_01)-128)*0.5)/128)+1;
  MAG_ASA[2] = (((MPU9250_Read(EXT_SENS_DATA_02)-128)*0.5)/128)+1;

	MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963); 
	MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_CNTL1); 
	MPU9250_Write(MPU9250_I2C_SLV0_DO,0X00);												//Power down magnetometer 
	MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0X81);
	HAL_Delay(50);

	MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963);
	MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_CNTL1); 
	MPU9250_Write(MPU9250_I2C_SLV0_DO,0X16);												//100hz 16bit
	MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0X81);
	HAL_Delay(50);

	MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963|0X80);
	MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_CNTL1); 
	MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0X81);
	HAL_Delay(50);

  SPI_Configuration(SPI_BAUDRATEPRESCALER_2);
	
}

/* MPU9250 Read & Write --------------------------------------------*/
void MPU9250_Write(uint8_t address,uint8_t CMD){
	SPI_TxBuff[0] = address;
	SPI_TxBuff[1] = CMD;
	CS_MP_L;
	HAL_SPI_Transmit(&hspi3,SPI_TxBuff,2,1);
	CS_MP_H;
	for(int i=0;i<50;i++){}	
}

uint8_t MPU9250_Read(uint8_t address){
	uint8_t return_value; 
	SPI_TxBuff[0] = (address|0x80);
	SPI_TxBuff[1] = 0;
 	CS_MP_L;
	HAL_SPI_TransmitReceive(&hspi3,SPI_TxBuff,SPI_MP_RxBuff.Byte,2,1);
	CS_MP_H;
	for(int i=0;i<50;i++){}	
	return_value = (SPI_MP_RxBuff.Byte[1]);
	return return_value ;
}

/* SPI3 Configuration----------------------------------------------*/
void SPI_Configuration(uint32_t BaudRatePrescaler)
{
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = BaudRatePrescaler;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi3);
}
