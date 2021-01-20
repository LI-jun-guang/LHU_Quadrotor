/*
	*@file : MPU9250
	*@author : Guang 
	*@date : 2018/08/11
	*
	*
	*
	*/
	
/* Includes -------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/*-----------------------------------------------------------------*/
#define CS_MP_L HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET)
#define CS_MP_H HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET)

#define MPU9250_CONFIG            0X1A
#define MPU9250_GYRO_CONFIG       0X1B
#define MPU9250_ACCEL_CONFIG      0X1C
#define MPU9250_ACCEL_CONFIG_2    0X1D
#define MPU9250_WHO_AM_I          0x75
#define MPU9250_USER_CTRL					0x6A
#define MPU9250_PWR_MGMT_1 				0x6B
#define MPU9250_PWR_MGMT_2 				0x6C
#define MPU9250_SIGNAL_PATH_RESET 0x68
#define MPU9250_FIFO_Enable				0x23
#define MPU9250_self_test				  0xE0

#define MPU9250_ACCEL_FS_2	 			0x00  
#define MPU9250_ACCEL_FS_4	 			0x08  
#define MPU9250_ACCEL_FS_8	 			0x10	
#define MPU9250_ACCEL_FS_16	 			0x18	

#define MPU9250_GYRO_FS_250 			0x00	
#define MPU9250_GYRO_FS_500 			0x08	
#define MPU9250_GYRO_FS_1000 			0x10	
#define MPU9250_GYRO_FS_2000 			0x18	

#define MPU9250_XG_OFFSET_H 			0x13 //GX_axis offset High_byte
#define MPU9250_XG_OFFSET_L 			0x14 //GX_axis offset Low_byte
#define MPU9250_YG_OFFSET_H 			0x15 //GY_axis offset High_byte
#define MPU9250_YG_OFFSET_L 			0x16 //GY_axis offset Low_byte
#define MPU9250_ZG_OFFSET_H				0x17 //GZ_axis offset High_byte
#define MPU9250_ZG_OFFSET_L 			0x18 //GZ_axis offset Low_byte

#define MPU9250_XA_OFFSET_H 			0x77 //AX_axis offset High_byte
#define MPU9250_XA_OFFSET_L 			0x78 //AX_axis offset Low_byte
#define MPU9250_YA_OFFSET_H			  0x7A //AY_axis offset High_byte
#define MPU9250_YA_OFFSET_L 			0x7B //AY_axis offset Low_byte
#define MPU9250_ZA_OFFSET_H 			0x7D //AZ_axis offset High_byte
#define MPU9250_ZA_OFFSET_L 			0x7E //AZ_axis offset Low_byte

#define MPU9250_ACCEL_XOUT_H 			0x3B
#define MPU9250_ACCEL_XOUT_L 			0x3C
#define MPU9250_ACCEL_YOUT_H 			0x3D
#define MPU9250_ACCEL_YOUT_L 			0x3E
#define MPU9250_ACCEL_ZOUT_H 			0x3F
#define MPU9250_ACCEL_ZOUT_L 			0x40

#define MPU9250_GYRO_XOUT_H 			0x43
#define MPU9250_GYRO_XOUT_L 			0x44
#define MPU9250_GYRO_YOUT_H 			0x45
#define MPU9250_GYRO_YOUT_L 			0x46
#define MPU9250_GYRO_ZOUT_H 			0x47
#define MPU9250_GYRO_ZOUT_L 			0x48

#define MPU9250_MAG_WAI  					0x00 //Return 0x48
#define MPU9250_MAG_INFO  				0x01
#define MPU9250_MAG_ST1  					0x02
#define MPU9250_MAG_ST2  					0x09
#define MPU9250_MAG_CNTL1  				0x0A
#define MPU9250_MAG_CNTL2  				0x0B
#define MPU9250_MAG_AK8963  			0x0C //AK8963 I2C Address
#define MPU9250_MAG_TS1  					0x0D
#define MPU9250_MAG_TS2  					0x0E
#define MPU9250_MAG_I2CDIS  			0x0F
#define MPU9250_MAG_ASAX 		    	0x10
#define MPU9250_MAG_ASAY 					0x11
#define MPU9250_MAG_ASAZ					0x12
#define MPU9250_MAG_REV						0x13

#define MPU9250_MAG_HXL  					0x03 //HX_axis offset Low_byte
#define MPU9250_MAG_HXH  					0x04 //HX_axis offset High_byte
#define MPU9250_MAG_HYL  					0x05 //HY_axis offset Low_byte
#define MPU9250_MAG_HYH  					0x06 //HY_axis offset High_byte
#define MPU9250_MAG_HZL  					0x07 //HZ_axis offset Low_byte
#define MPU9250_MAG_HZH  					0x08 //HZ_axis offset High_byte

#define MPU9250_I2C_MST_CTRL			0X24

#define MPU9250_I2C_SLV0_ADDR			0X25
#define MPU9250_I2C_SLV0_REG			0X26
#define MPU9250_I2C_SLV0_CTRL			0X27
#define MPU9250_I2C_SLV0_DO				0X63

#define MPU9250_I2C_SLV4_ADDR			0X31
#define MPU9250_I2C_SLV4_REG			0X32
#define MPU9250_I2C_SLV4_DO				0X33
#define MPU9250_I2C_SLV4_CTRL			0X34
#define MPU9250_I2C_SLV4_DI				0X35

#define EXT_SENS_DATA_00 					0x49
#define EXT_SENS_DATA_01 					0x4A
#define EXT_SENS_DATA_02 					0x4B
#define EXT_SENS_DATA_03 					0x4C
#define EXT_SENS_DATA_04 					0x4D
#define EXT_SENS_DATA_05          0x4E
#define EXT_SENS_DATA_06 					0x4F
#define EXT_SENS_DATA_07 					0x50
#define EXT_SENS_DATA_08 					0x51
#define EXT_SENS_DATA_09 					0x52
#define EXT_SENS_DATA_10 					0x53
#define EXT_SENS_DATA_11 					0x54
#define EXT_SENS_DATA_12 					0x55
#define EXT_SENS_DATA_13 					0x56
#define EXT_SENS_DATA_14 					0x57
#define EXT_SENS_DATA_15 					0x58
#define EXT_SENS_DATA_16 					0x59
#define EXT_SENS_DATA_17 					0x5A
#define EXT_SENS_DATA_18 					0x5B
#define EXT_SENS_DATA_19 					0x5C
#define EXT_SENS_DATA_20					0x5D
#define EXT_SENS_DATA_21 					0x5E
#define EXT_SENS_DATA_22 					0x5F
#define EXT_SENS_DATA_23 					0x60

typedef union
{
	uint8_t Byte[2];
	int16_t data;
}MPU9250_Raw;


void MPU9250_Configuration(void);
void MPU9250_Write(uint8_t address,uint8_t CMD);
uint8_t MPU9250_Read(uint8_t address);













