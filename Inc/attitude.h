	/*
	*@file : attitude
	*@author : Guang 
	*@date : 2017/08/08
	*@drief : MPU6500«ººA¿Ä¦X
	*
	*
	*
	*/
	
/* Includes --------------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/*EEPRON Address------------------------------------------*/
/*0x26~0x2 is Accel offset and Magn hard iron eeprom address*/
#define Accel_offset_X_Adderss		0x26
#define Accel_offset_Y_Adderss		0x27				
#define Accel_offset_Z_Adderss		0x28

#define Magn_hard_iron_X_Adderss	0x29
#define Magn_hard_iron_Y_Adderss	0x30				
#define Magn_hard_iron_Z_Adderss	0x31


/*-------------------------------------------------------*/

/************************************

  Axis[0]= X axis 
  Axis[1]= Y axis 
	Axis[2]= Z axis 
	
	Byte[1] = IMU High Byte
  Byte[0] = IMU  LOW Byte

************************************/
typedef union 
 {
	 int16_t RawData;
	 uint8_t Bytes[2];

 } attitude;
typedef  struct
 {
   attitude Axis[3];     
	 
 }	sensor;

	typedef struct
	{
		float new_raw;
		float lastdata;
		float omega;	
		float angle;
		float Lowpass_output;
		
		float  offset;
		float  Calibration;
	  float  ZERO_X[6];
		float  ZERO_Y[6];
		float  ZERO_Z[6];
		
	} MPU9250;
	
	typedef struct
	{
		float A[18];
		float AT[18];
		float B[6];
		float X[3];
		float RaW[18];
		float ATMAI[9];
		float ATMA[18];
		float ATMAIAT[18];
	}matrix;
	

	void Gyro_data(void);
	void Accel_data(void);
	void Magn_data(void);
  void MPU9250_Gyro_calibr(void);
 	void MPU9250_Accle_calibr(void);
	void MPU9250_Magn_calibr(void);
	void MPU9250_Magn_Levelcor(void);
	
	
