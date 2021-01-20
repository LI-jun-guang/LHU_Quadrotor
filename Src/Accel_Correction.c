#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "math_helper.h"
#include "MPU9250.h"
#include "attitude.h"
#include "LED.h"

#define Receiver_accleCor_Lock     R12D[8]   
#define Receiver_accleCor_check    R12D[9]

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

extern sensor IMU;
extern MPU9250 Gyro[3];
extern MPU9250 Accel[3];
extern uint16_t R12D[12];
extern uint8_t First_menu;

void Accle_cal(void)
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

		
		