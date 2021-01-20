	/*
	*@file : attitude
	*@author : Guang 
	*@date : 2017/08/08
	*@drief : MPU9250姿態融合
	*
	*
	*
	*/
	/*Includes--------------------------------------------------------------------------------*/
//Accel X Axis:-85.849
//Accel Y Axis:-0.129
//Accel Z Axis:-355.863
	#include "stm32f4xx_hal.h"
	#include "math_helper.h"
	#include "arm_math.h"
	#include "math.h"
	#include "MPU9250.h"
	#include "attitude.h"
	#include "LED.h"
	
	#define RAD_TO_DEG        (180/3.14f)      
	#define ACCE_Sensitivity  0.00006103515625f //加速度計靈敏度
	#define GYRO_Sensitivity  0.0076335877862595f   //陀螺儀靈敏度
	#define MAGN_Sensitivity	0.15f		    		  //磁力計靈敏度
	
	#define Accel_X_offset      -31.78f								//加速度計X軸偏移值
	#define Accel_Y_offset       2.14f							  //加速度計Y軸偏移值
	#define Accel_Z_offset      -340.55f								//加速度計Z軸偏移值

//	#define Accel_X_offset      0								//加速度計X軸偏移值
//	#define Accel_Y_offset      0							  //加速度計Y軸偏移值
//	#define Accel_Z_offset      0								//加速度計Z軸偏移值

	#define Magn_LimitMax    ( 170.f - magn_zeroPoint)	//Magn Angle limit Max
 	#define Magn_LimitMin    (-170.f - magn_zeroPoint)	//Magn Angle limit Min
	#define MAGN_LimitGain     0.7f   //Magn Angle limit Range 70%

	#define Gyro_dt 0.002;											//Gyro integral time
	
	#define Receiver_accleCor_Lock     R12D[8]   
	#define Receiver_accleCor_check    R12D[9]

	#define Lowpass_Filter_Gain  0.1 
	
	sensor IMU;
	MPU9250 Gyro[3];
	MPU9250 Accel[3];
	MPU9250 Magn[3];
	
	matrix ZERO;
	uint8_t Accel_Cal_status = 0 ,Accel_lock=0,ACL_flag;
	uint16_t j = 0;
	arm_matrix_instance_f32 A;      
	arm_matrix_instance_f32 AT;     
	arm_matrix_instance_f32 ATMA;
	arm_matrix_instance_f32 ATMAI;
	arm_matrix_instance_f32 ATMAIAT;
	arm_matrix_instance_f32 B;
	arm_matrix_instance_f32 X;
	uint32_t srcRows, srcColumns;  
	arm_status status;
		
	extern float MAG_ASA[3];
	extern int16_t MPU9250_cal;
	extern uint16_t R12D[12];
	extern uint8_t First_menu;
	extern float MAG_ASA[3]; 				//AK8963 asax,asay,asaz data
	
	float Magn_AngleMax,Magn_AngleMin;
	float mx,my; 
	float MAG_pitch,MAG_roll;
	float magn_zeroPoint;
	uint8_t mag_zeroflag = 1;
	uint16_t magcal = 0;
	/*---------------------------------------------------------------------------------*/
	float accelz[3];






  /*--------------------------------MPU9250讀取原始值--------------------------------*/
	void Gyro_data(void)
	{
		/*陀螺儀原始值*/
		IMU.Axis[0].Bytes[1] = MPU9250_Read(0x43); //X Axis H_Byte
		IMU.Axis[0].Bytes[0] = MPU9250_Read(0x44); //X Axis L_Byte
		IMU.Axis[1].Bytes[1] = MPU9250_Read(0x45); //Y Axis H_Byte
		IMU.Axis[1].Bytes[0] = MPU9250_Read(0x46); //Y Axis L_Byte
		IMU.Axis[2].Bytes[1] = MPU9250_Read(0x47); //Z Axis H_Byte
		IMU.Axis[2].Bytes[0] = MPU9250_Read(0x48); //Z Axis L_Byte
		
		
		/*陀螺儀偏移值正*/
		Gyro[0].new_raw = (IMU.Axis[0].RawData - Gyro[0].offset)*GYRO_Sensitivity;
		Gyro[1].new_raw = (IMU.Axis[1].RawData - Gyro[1].offset)*GYRO_Sensitivity;
		Gyro[2].new_raw = (IMU.Axis[2].RawData - Gyro[2].offset)*GYRO_Sensitivity;

//		/*陀螺儀角度*/
//		Gyro[0].angle += Gyro[0].new_raw*Gyro_dt; 
//		Gyro[1].angle += Gyro[1].new_raw*Gyro_dt; 
//		Gyro[2].angle += (Gyro[2].new_raw-0.000003)*Gyro_dt;
		
		/*-------------------------------------------------------------------------------------------------------------------*/
		Gyro[0].Lowpass_output = (Gyro[0].new_raw*0.3f)+(Gyro[0].lastdata*(0.7f));               
		Gyro[1].Lowpass_output = (Gyro[1].new_raw*0.3f)+(Gyro[1].lastdata*(0.7f));	
		Gyro[2].Lowpass_output = (Gyro[2].new_raw*0.2f)+(Gyro[2].lastdata*(0.8f));	
		
		/*陀螺儀角度*/
		Gyro[0].angle += Gyro[0].Lowpass_output*Gyro_dt; 
		Gyro[1].angle += Gyro[1].Lowpass_output*Gyro_dt; 
		Gyro[2].angle += (Gyro[2].Lowpass_output-0.0003)*Gyro_dt;
		
		Gyro[0].lastdata = Gyro[0].Lowpass_output;
		Gyro[1].lastdata = Gyro[1].Lowpass_output;
		Gyro[2].lastdata = Gyro[2].Lowpass_output;
	}
		 
	void Accel_data(void)
	{
		/*加速度原始值*/
		IMU.Axis[0].Bytes[1] = MPU9250_Read(0x3B);
		IMU.Axis[0].Bytes[0] = MPU9250_Read(0x3C);
		IMU.Axis[1].Bytes[1] = MPU9250_Read(0x3D);
		IMU.Axis[1].Bytes[0] = MPU9250_Read(0x3E);
		IMU.Axis[2].Bytes[1] = MPU9250_Read(0x3F);
		IMU.Axis[2].Bytes[0] = MPU9250_Read(0x40);
		
		/*加速度偏移值校正*/
		Accel[0].new_raw = (IMU.Axis[0].RawData - Accel_X_offset)*ACCE_Sensitivity;
		Accel[1].new_raw = (IMU.Axis[1].RawData - Accel_Y_offset)*ACCE_Sensitivity;
		Accel[2].new_raw = (IMU.Axis[2].RawData - Accel_Z_offset)*ACCE_Sensitivity;
		
		/*加速度角度計算*/
		Accel[0].angle = (atan2(Accel[1].new_raw,Accel[2].new_raw)*RAD_TO_DEG);
		Accel[1].angle = (atan2((-Accel[0].new_raw),Accel[2].new_raw)*RAD_TO_DEG);
			
		/*-------------------------------------------------------------------------------------------------------------------*/
		Accel[0].Lowpass_output = (Accel[0].new_raw*Lowpass_Filter_Gain )+(Accel[0].lastdata*(1-Lowpass_Filter_Gain ));
		Accel[1].Lowpass_output = (Accel[1].new_raw*Lowpass_Filter_Gain )+(Accel[1].lastdata*(1-Lowpass_Filter_Gain ));	
		Accel[2].Lowpass_output = (Accel[2].new_raw*Lowpass_Filter_Gain )+(Accel[2].lastdata*(1-Lowpass_Filter_Gain ));	
		
		accelz[0] += ((Accel[2].new_raw)*9.8*0.001)*0.002*0.01;
		
		accelz[1] += accelz[0]*0.002;
		
		Accel[0].lastdata = Accel[0].Lowpass_output;
		Accel[1].lastdata = Accel[1].Lowpass_output;
		Accel[2].lastdata = Accel[2].Lowpass_output;
		
	}
	
	void Magn_data(void)
	{

		MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963|0X80); //Set the I2C slave address of AK8963 and set for read.
		MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_HXL); 					//Read MAG_HXL~MAG_HZH and ST2
		MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0x87);										//READ 7 byte data

		if(!(MPU9250_Read(EXT_SENS_DATA_06)&0X08))
    {
			IMU.Axis[0].Bytes[0] = MPU9250_Read(EXT_SENS_DATA_00); 
	    IMU.Axis[0].Bytes[1] = MPU9250_Read(EXT_SENS_DATA_01);
	
	    IMU.Axis[1].Bytes[0] = MPU9250_Read(EXT_SENS_DATA_02);
    	IMU.Axis[1].Bytes[1] = MPU9250_Read(EXT_SENS_DATA_03);
		
	    IMU.Axis[2].Bytes[0] = MPU9250_Read(EXT_SENS_DATA_04);
	    IMU.Axis[2].Bytes[1] = MPU9250_Read(EXT_SENS_DATA_05);
		}
		
		Magn[0].new_raw = (float)(((IMU.Axis[0].RawData * MAG_ASA[0] * MAGN_Sensitivity * 10) - 27.2)*0.96f);
		Magn[1].new_raw = (float)(((IMU.Axis[1].RawData * MAG_ASA[1] * MAGN_Sensitivity * 10) - 110.8)*1.16f);
		Magn[2].new_raw = (float)(((IMU.Axis[2].RawData * MAG_ASA[2] * MAGN_Sensitivity * 10) - (-154.1))*1.02f);

		MAG_pitch = atan2(Accel[1].new_raw,sqrt((Accel[0].new_raw*Accel[0].new_raw)+(Accel[2].new_raw*Accel[2].new_raw))); 
		MAG_roll = atan2(-Accel[0].new_raw,sqrt((Accel[1].new_raw*Accel[1].new_raw)+(Accel[2].new_raw*Accel[2].new_raw))); 
	
		mx =  (Magn[0].new_raw * cos(MAG_pitch)) + (Magn[1].new_raw * sin(MAG_roll) * sin(MAG_pitch)) - (Magn[2].new_raw * cos(MAG_roll) * sin(MAG_pitch));
		my =  (Magn[1].new_raw * cos(MAG_roll))+ (Magn[2].new_raw * sin(MAG_roll));
			
		Magn[2].angle = (atan2(my,mx)*RAD_TO_DEG); 
	}
		
  /*----------------------------------------------------------------*/
	void MPU9250_Gyro_calibr(void)
	{
		/*陀螺儀原始值*/
		IMU.Axis[0].Bytes[1] = MPU9250_Read(0x43); //X Axis H_Byte
		IMU.Axis[0].Bytes[0] = MPU9250_Read(0x44); //X Axis L_Byte
		IMU.Axis[1].Bytes[1] = MPU9250_Read(0x45); //Y Axis H_Byte
		IMU.Axis[1].Bytes[0] = MPU9250_Read(0x46); //Y Axis L_Byte
		IMU.Axis[2].Bytes[1] = MPU9250_Read(0x47); //Z Axis H_Byte
		IMU.Axis[2].Bytes[0] = MPU9250_Read(0x48); //Z Axis L_Byte
  
		if(MPU9250_cal > 1000)
		{
			Gyro[0].offset += IMU.Axis[0].RawData;
			Gyro[1].offset += IMU.Axis[1].RawData;
			Gyro[2].offset += IMU.Axis[2].RawData;
		}
		if(MPU9250_cal== 3000)
		{
			Gyro[0].offset /= 2000;
			Gyro[1].offset /= 2000;
			Gyro[2].offset /= 2000;
		}
		MPU9250_cal++;
		HAL_Delay(1);
	}
	
	void MPU9250_Accle_calibr(void)
	{
	/*加速度原始值*/
		if(Receiver_accleCor_check >1600 && Receiver_accleCor_Lock >1600)
		{
			if(ACL_flag == 4 )
			{
				if(j<2000)
				{	
					j++; 
					ACL_flag = 0;
					IMU.Axis[0].Bytes[1] = MPU9250_Read(0x3B);
					IMU.Axis[0].Bytes[0] = MPU9250_Read(0x3C);
					
					IMU.Axis[1].Bytes[1] = MPU9250_Read(0x3D);
					IMU.Axis[1].Bytes[0] = MPU9250_Read(0x3E);

					IMU.Axis[2].Bytes[1] = MPU9250_Read(0x3F);
					IMU.Axis[2].Bytes[0] = MPU9250_Read(0x40);
					
					Accel[0].Calibration  += IMU.Axis[0].RawData;  //X軸累加
					Accel[1].Calibration  += IMU.Axis[1].RawData;  //Y軸累加
					Accel[2].Calibration  += IMU.Axis[2].RawData;  //Z軸累加
					
					RGB(1000,100,1000);//紅色LED
					Accel_lock = 1;
					}
				else if(j>=2000 && Accel_lock==1)
				{
					switch(Accel_Cal_status)
					{
						case 0:
							printf("\nAccel_Cal_status:%d\n",Accel_Cal_status);
							//第一次姿態資料
							Accel[0].ZERO_X[0] = Accel[0].Calibration/2000;
							Accel[1].ZERO_Y[0] = Accel[1].Calibration/2000;
							Accel[2].ZERO_Z[0] = Accel[2].Calibration/2000;
						
							Accel_lock = 0;
							RGB(100,1000,1000); //綠色LED
							HAL_Delay(1000);
							break;
						
						case 1:
							printf("\nAccel_Cal_status:%d\n",Accel_Cal_status);
							//第二次姿態資料
							Accel[0].ZERO_X[1] = Accel[0].Calibration/2000;
							Accel[1].ZERO_Y[1] = Accel[1].Calibration/2000;
							Accel[2].ZERO_Z[1] = Accel[2].Calibration/2000;
						
							Accel_lock = 0;
							RGB(100,1000,1000); //綠色LED
							HAL_Delay(1000);
							break;
						
						case 2:
							printf("\nAccel_Cal_status:%d\n",Accel_Cal_status);
							//第三次姿態資料
							Accel[0].ZERO_X[2] = Accel[0].Calibration/2000;
							Accel[1].ZERO_Y[2] = Accel[1].Calibration/2000;
							Accel[2].ZERO_Z[2] = Accel[2].Calibration/2000;
						
							Accel_lock = 0;
							RGB(100,1000,1000); //綠色LED
							HAL_Delay(1000);
							break;
						
						case 3:
							printf("\nAccel_Cal_status:%d\n",Accel_Cal_status);
							//第四次姿態資料
							Accel[0].ZERO_X[3] = Accel[0].Calibration/2000;
							Accel[1].ZERO_Y[3] = Accel[1].Calibration/2000;
							Accel[2].ZERO_Z[3] = Accel[2].Calibration/2000;
						
							Accel_lock = 0;
							RGB(100,1000,1000); //綠色LED
							HAL_Delay(1000);
							break;
						
						case 4:
							printf("\nAccel_Cal_status:%d\n",Accel_Cal_status);
							//第五次姿態資料
							Accel[0].ZERO_X[4] = Accel[0].Calibration/2000;
							Accel[1].ZERO_Y[4] = Accel[1].Calibration/2000;
							Accel[2].ZERO_Z[4] = Accel[2].Calibration/2000;
						
							Accel_lock = 0;
							RGB(100,1000,1000); //綠色LED
							HAL_Delay(1000);
							break;
						
						case 5:
							printf("\nAccel_Cal_status:%d\n",Accel_Cal_status);
							//第六次姿態資料
							Accel[0].ZERO_X[5] = Accel[0].Calibration/2000;
							Accel[1].ZERO_Y[5] = Accel[1].Calibration/2000;
							Accel[2].ZERO_Z[5] = Accel[2].Calibration/2000;
						
							Accel_lock = 0;
							RGB(100,1000,1000); //綠色LED
							HAL_Delay(1000);
							break;
				
						default:
							break;
					}
				}
			}
		}
		
		else if(Receiver_accleCor_check >1600 && Receiver_accleCor_Lock <1600 && j==2000 && Accel_lock==0) 
		{
			printf("\nReceiver_accleCor_check:ON Receiver_accleCor_Lock:OFF\n");
			RGB(1000,1000,100); //藍色LED
			j = 0;
			Accel_Cal_status +=1;
			Accel[0].Calibration  = Accel[1].Calibration = Accel[2].Calibration = 0;	
		}

		if(Accel_Cal_status== 6)
		{
			
	//		Accel[0].ZERO_X[0] =  -965;
	//		Accel[0].ZERO_X[1] = 15749;
	//		Accel[0].ZERO_X[2] =  -17021;
	//		Accel[0].ZERO_X[3] =-1388;
	//		Accel[0].ZERO_X[4] = -894;
	//		Accel[0].ZERO_X[5] = -1170; 
	//		
	//		Accel[1].ZERO_Y[0] =  425; 
	//		Accel[1].ZERO_Y[1] = 24;
	//		Accel[1].ZERO_Y[2] = 514;
	//		Accel[1].ZERO_Y[3] =  -16135;
	//		Accel[1].ZERO_Y[4] =  16636;
	//		Accel[1].ZERO_Y[5] =  23;
	//		
	//		Accel[2].ZERO_Z[0] =  16200;
	//		Accel[2].ZERO_Z[1] =  -253;
	//		Accel[2].ZERO_Z[2] =  211;
	//		Accel[2].ZERO_Z[3] =  -77;
	//		Accel[2].ZERO_Z[4] =  -454;
	//		Accel[2].ZERO_Z[5] = -16841;
			
			ZERO.A[0] = 2*(Accel[0].ZERO_X[0]-Accel[0].ZERO_X[1]); //2*(X1-X2)
			ZERO.A[1] = 2*(Accel[1].ZERO_Y[0]-Accel[1].ZERO_Y[1]); //2*(Y1-Y2)
			ZERO.A[2] = 2*(Accel[2].ZERO_Z[0]-Accel[2].ZERO_Z[1]); //2*(Z1-Z2)
			
			ZERO.A[3] = 2*(Accel[0].ZERO_X[1]-Accel[0].ZERO_X[2]); //2*(X2-X3)
			ZERO.A[4] = 2*(Accel[1].ZERO_Y[1]-Accel[1].ZERO_Y[2]); //2*(Y2-Y3)
			ZERO.A[5] = 2*(Accel[2].ZERO_Z[1]-Accel[2].ZERO_Z[2]); //2*(Z2-Z3)
			
			ZERO.A[6] = 2*(Accel[0].ZERO_X[2]-Accel[0].ZERO_X[3]); //2*(X3-X4)
			ZERO.A[7] = 2*(Accel[1].ZERO_Y[2]-Accel[1].ZERO_Y[3]); //2*(Y3-Y4)
			ZERO.A[8] = 2*(Accel[2].ZERO_Z[2]-Accel[2].ZERO_Z[3]); //2*(Z3-Z4)
			
			ZERO.A[9]  = 2*(Accel[0].ZERO_X[3]-Accel[0].ZERO_X[4]); //2*(X4-X5)
			ZERO.A[10] = 2*(Accel[1].ZERO_Y[3]-Accel[1].ZERO_Y[4]); //2*(Y4-Y5)
			ZERO.A[11] = 2*(Accel[2].ZERO_Z[3]-Accel[2].ZERO_Z[4]); //2*(Z4-Z5)
			
			ZERO.A[12] = 2*(Accel[0].ZERO_X[4]-Accel[0].ZERO_X[5]); //2*(X5-X6)
			ZERO.A[13] = 2*(Accel[1].ZERO_Y[4]-Accel[1].ZERO_Y[5]); //2*(Y5-Y6)
			ZERO.A[14] = 2*(Accel[2].ZERO_Z[4]-Accel[2].ZERO_Z[5]); //2*(Z5-Z6)
			
			ZERO.A[15] = 2*(Accel[0].ZERO_X[5]-Accel[0].ZERO_X[0]); //2*(X6-X1)
			ZERO.A[16] = 2*(Accel[1].ZERO_Y[5]-Accel[1].ZERO_Y[0]); //2*(Y6-Y1)
			ZERO.A[17] = 2*(Accel[2].ZERO_Z[5]-Accel[2].ZERO_Z[0]); //2*(Z6-Z1)
			
			ZERO.B[0] = (Accel[0].ZERO_X[0]*Accel[0].ZERO_X[0]) + (Accel[1].ZERO_Y[0]*Accel[1].ZERO_Y[0]) + (Accel[2].ZERO_Z[0]*Accel[2].ZERO_Z[0])-(Accel[0].ZERO_X[1]*Accel[0].ZERO_X[1]) - (Accel[1].ZERO_Y[1]*Accel[1].ZERO_Y[1]) - (Accel[2].ZERO_Z[1]*Accel[2].ZERO_Z[1]);    
			ZERO.B[1] = (Accel[0].ZERO_X[1]*Accel[0].ZERO_X[1]) + (Accel[1].ZERO_Y[1]*Accel[1].ZERO_Y[1]) + (Accel[2].ZERO_Z[1]*Accel[2].ZERO_Z[1])-(Accel[0].ZERO_X[2]*Accel[0].ZERO_X[2]) - (Accel[1].ZERO_Y[2]*Accel[1].ZERO_Y[2]) - (Accel[2].ZERO_Z[2]*Accel[2].ZERO_Z[2]); 
			ZERO.B[2] = (Accel[0].ZERO_X[2]*Accel[0].ZERO_X[2]) + (Accel[1].ZERO_Y[2]*Accel[1].ZERO_Y[2]) + (Accel[2].ZERO_Z[2]*Accel[2].ZERO_Z[2])-(Accel[0].ZERO_X[3]*Accel[0].ZERO_X[3]) - (Accel[1].ZERO_Y[3]*Accel[1].ZERO_Y[3]) - (Accel[2].ZERO_Z[3]*Accel[2].ZERO_Z[3]); 
			ZERO.B[3] = (Accel[0].ZERO_X[3]*Accel[0].ZERO_X[3]) + (Accel[1].ZERO_Y[3]*Accel[1].ZERO_Y[3]) + (Accel[2].ZERO_Z[3]*Accel[2].ZERO_Z[3])-(Accel[0].ZERO_X[4]*Accel[0].ZERO_X[4]) - (Accel[1].ZERO_Y[4]*Accel[1].ZERO_Y[4]) - (Accel[2].ZERO_Z[4]*Accel[2].ZERO_Z[4]);    
			ZERO.B[4] = (Accel[0].ZERO_X[4]*Accel[0].ZERO_X[4]) + (Accel[1].ZERO_Y[4]*Accel[1].ZERO_Y[4]) + (Accel[2].ZERO_Z[4]*Accel[2].ZERO_Z[4])-(Accel[0].ZERO_X[5]*Accel[0].ZERO_X[5]) - (Accel[1].ZERO_Y[5]*Accel[1].ZERO_Y[5]) - (Accel[2].ZERO_Z[5]*Accel[2].ZERO_Z[5]); 
			ZERO.B[5] = (Accel[0].ZERO_X[5]*Accel[0].ZERO_X[5]) + (Accel[1].ZERO_Y[5]*Accel[1].ZERO_Y[5]) + (Accel[2].ZERO_Z[5]*Accel[2].ZERO_Z[5])-(Accel[0].ZERO_X[0]*Accel[0].ZERO_X[0]) - (Accel[1].ZERO_Y[0]*Accel[1].ZERO_Y[0]) - (Accel[2].ZERO_Z[0]*Accel[2].ZERO_Z[0]); 
			
		 Accel_Cal_status++;
	 }
		
		if(Accel_Cal_status == 7 ) 
		{
			/* Initialise A Matrix Instance with numRows, numCols and data array(A_f32) */
			srcRows = 6;
			srcColumns = 3;
			arm_mat_init_f32(&A, srcRows, srcColumns, (float32_t *)ZERO.A);
			/* Initialise Matrix Instance AT with numRows, numCols and data array(AT_f32) */
			srcRows = 3;
			srcColumns = 6;
			arm_mat_init_f32(&AT, srcRows, srcColumns, (float32_t *)ZERO.AT);
			/* calculation of A transpose */
			status = arm_mat_trans_f32(&A, &AT);
			/* Initialise ATMA Matrix Instance with numRows, numCols and data array(ATMA_f32) */
			srcRows = 3;
			srcColumns = 3;
			arm_mat_init_f32(&ATMA, srcRows, srcColumns,(float32_t *)ZERO.ATMA);
			/* calculation of AT Multiply with A */
			status = arm_mat_mult_f32(&AT, &A, &ATMA);
			/* Initialise ATMAI Matrix Instance with numRows, numCols and data array(ATMAI_f32) */
			srcRows = 3;
			srcColumns = 3;
			arm_mat_init_f32(&ATMAI, srcRows, srcColumns,(float32_t *)ZERO.ATMAI);
			
			srcRows = 3;
			srcColumns = 6;
			arm_mat_init_f32(&ATMAIAT, srcRows, srcColumns,(float32_t *)ZERO.ATMAIAT);
			
			/* calculation of Inverse((Transpose(A) * A) */
			status = arm_mat_inverse_f32(&ATMA, &ATMAI);
			/* calculation of (Inverse((Transpose(A) * A)) *  Transpose(A)) */
			status = arm_mat_mult_f32(&ATMAI, &AT, &ATMAIAT);
			/* Initialise B Matrix Instance with numRows, numCols and data array(B_f32) */
			srcRows = 6;
			srcColumns = 1;
			arm_mat_init_f32(&B, srcRows, srcColumns, (float32_t *)ZERO.B);
			/* Initialise X Matrix Instance with numRows, numCols and data array(X_f32) */
			
			srcRows = 3;
			srcColumns = 1;
			arm_mat_init_f32(&X, srcRows, srcColumns, (float32_t *)ZERO.X);
			/* calculation ((Inverse((Transpose(A) * A)) *  Transpose(A)) * B) */
			status = arm_mat_mult_f32(&ATMAIAT, &B, &X);
			
			printf("\nAccel X Axis:%.3f\n",ZERO.X[0]);
			printf("Accel Y Axis:%.3f\n",ZERO.X[1]);
			printf("Accel Z Axis:%.3f\n",ZERO.X[2]);
			Accel_Cal_status++;
			First_menu = 0;  
		}
	}
	
	void MPU9250_Magn_calibr(void)
	{
		uint16_t ii = 0, sample_count = 0;
		int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
		int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

		sample_count = 2000;
			
		for(ii = 0; ii < sample_count; ii++) 
		{
			MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963|0X80); //Set the I2C slave address of AK8963 and set for read.
			MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_HXL); 					//Read MAG_HXL~MAG_HZH and ST2
			MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0x87);										//READ 7 byte data
			HAL_Delay(5);
			if(!(MPU9250_Read(EXT_SENS_DATA_06)&0X08))
			{
				IMU.Axis[0].Bytes[0] = MPU9250_Read(EXT_SENS_DATA_00); 
				IMU.Axis[0].Bytes[1] = MPU9250_Read(EXT_SENS_DATA_01);
		
				IMU.Axis[1].Bytes[0] = MPU9250_Read(EXT_SENS_DATA_02);
				IMU.Axis[1].Bytes[1] = MPU9250_Read(EXT_SENS_DATA_03);
			
				IMU.Axis[2].Bytes[0] = MPU9250_Read(EXT_SENS_DATA_04);
				IMU.Axis[2].Bytes[1] = MPU9250_Read(EXT_SENS_DATA_05);
			}
			mag_temp[0] = IMU.Axis[0].RawData ;
			mag_temp[1] = IMU.Axis[1].RawData ;
			mag_temp[2] = IMU.Axis[2].RawData ;  

			for (int jj = 0; jj < 3; jj++) 
			{
				if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
			HAL_Delay(15);
		}
		
	  // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    Magn[0].offset = (float)(mag_bias[0] * MAGN_Sensitivity * MAG_ASA[0] * 10);  // save mag biases in G for main program
    Magn[1].offset = (float)(mag_bias[1] * MAGN_Sensitivity * MAG_ASA[1] * 10);   
    Magn[2].offset = (float)(mag_bias[2] * MAGN_Sensitivity * MAG_ASA[2] * 10);  
    
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

		float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    Magn[0].Calibration = avg_rad/((float)mag_scale[0]);
    Magn[1].Calibration = avg_rad/((float)mag_scale[1]);
    Magn[2].Calibration = avg_rad/((float)mag_scale[2]);
 
		First_menu = 0;
	}
	
	void MPU9250_Magn_Levelcor(void)// 準位校正
	{
		MPU9250_Write(MPU9250_I2C_SLV0_ADDR,MPU9250_MAG_AK8963|0X80); //Set the I2C slave address of AK8963 and set for read.
		MPU9250_Write(MPU9250_I2C_SLV0_REG,MPU9250_MAG_HXL); 					//Read MAG_HXL~MAG_HZH and ST2
		MPU9250_Write(MPU9250_I2C_SLV0_CTRL,0x87);										//READ 7 byte data

		if(!(MPU9250_Read(EXT_SENS_DATA_06)&0X08))
    {
			IMU.Axis[0].Bytes[0] = MPU9250_Read(EXT_SENS_DATA_00); 
	    IMU.Axis[0].Bytes[1] = MPU9250_Read(EXT_SENS_DATA_01);
	
	    IMU.Axis[1].Bytes[0] = MPU9250_Read(EXT_SENS_DATA_02);
    	IMU.Axis[1].Bytes[1] = MPU9250_Read(EXT_SENS_DATA_03);
		
	    IMU.Axis[2].Bytes[0] = MPU9250_Read(EXT_SENS_DATA_04);
	    IMU.Axis[2].Bytes[1] = MPU9250_Read(EXT_SENS_DATA_05);
		}
		
		Magn[0].new_raw = (float)(((IMU.Axis[0].RawData * MAG_ASA[0] * MAGN_Sensitivity * 10) - 27.2)*0.96f);
		Magn[1].new_raw = (float)(((IMU.Axis[1].RawData * MAG_ASA[1] * MAGN_Sensitivity * 10) - 110.8)*1.16f);
		Magn[2].new_raw = (float)(((IMU.Axis[2].RawData * MAG_ASA[2] * MAGN_Sensitivity * 10) - (-154.1))*1.02f);
		
		
		MAG_pitch = atan2(Accel[1].new_raw,sqrt((Accel[0].new_raw*Accel[0].new_raw)+(Accel[2].new_raw*Accel[2].new_raw))); 
		MAG_roll = atan2(-Accel[0].new_raw,sqrt((Accel[1].new_raw*Accel[1].new_raw)+(Accel[2].new_raw*Accel[2].new_raw))); 
			
		mx =  (Magn[0].new_raw * cos(MAG_pitch)) + (Magn[1].new_raw * sin(MAG_roll) * sin(MAG_pitch)) - (Magn[2].new_raw * cos(MAG_roll) * sin(MAG_pitch));
		my =  (Magn[1].new_raw * cos(MAG_roll))+ (Magn[2].new_raw * sin(MAG_roll));
			
		Magn[2].angle = atan2(my,mx)*RAD_TO_DEG;
		
		HAL_Delay(12);
		
		if(magcal < 1001) magn_zeroPoint += Magn[2].angle;

		if(magcal == 1000) magn_zeroPoint /= 1000;
		

		magcal++;
	}		
		
				