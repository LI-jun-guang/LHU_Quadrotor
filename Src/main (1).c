
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "LED.h"
#include "PID.h"
#include "Math.h"
#include "eeprom.h"
#include "MS5611.h"
#include "Kalman.h"
#include "Battery.h"
#include "MPU9250.h"
#include "attitude.h"
#include "arm_math.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>


#define Receiver_PWMMax 1915U
#define Receiver_PWMMin 1086U

#define Receiver_ideal_PWMMin 1000U
#define Receiver_ideal_PWMMax 2000U
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t throttle; 
int8_t Fly_Start = 0; //飛行狀態
 
/*kalman 變數*/
struct Kalman kalmanX,kalmanY; 
float kalman_Pitch,kalman_Roll;   
float dt =0.004;
	
int16_t MPU9250_cal=0;
int8_t loop_10Hz_flag=0,loop_500Hz_flag=0,loop_100Hz_flag=0,loop_50Hz_flag=0;
int16_t Fly_flag=0;

char     Tx_buff[200]={0};
char 		 UART_Rx_buff[3]; //Receiver dma data
char     *temrxdata = UART_Rx_buff; //char 指標
uint8_t  UART_Rx_COUNT=0,UART_Rx_BUFFSIZE=3;

extern uint16_t R12D[12];

extern sensor IMU;
extern MPU9250 Gyro[3];
extern MPU9250 Accel[3];
extern MPU9250 Magn[3];
extern Data MS5611;

extern PID_TypeDef PID_Roll; 
extern PID_TypeDef PID_Pitch;
extern PID_TypeDef PID_Yaw;
extern PID_TypeDef PID_Roll_Angle;
extern PID_TypeDef PID_Pitch_Angle;
extern PID_TypeDef PID_Yaw_Angle;

extern PID_TypeDef PID_Throttle;

extern 	 ADC_battery Battery;

/************************************************/

float magn_tsst = 0;
extern float mx,my; 
extern float MAG_pitch,MAG_roll;
extern float magn_zeroPoint;
extern uint8_t mag_zeroflag;
extern uint16_t magcal;
extern float magn_zeroPoint;
/************************************************/
float MS5611_actual_pressure, MS5611_actual_pressure_slow, MS5611_actual_pressure_fast, MS5611_actual_pressure_diff;
float MS5611_rotating_mem_actual;
int32_t MS5611_total_avarage,MS5611_rotating_mem[50];
uint8_t MS5611_rotating_mem_location,MS5611_Flag = 0,MS5611_D1_COUNT=0,MS5611_D2_COUNT=0;
uint8_t Auto_attitude_flag=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Main_meun(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/*UART副程式------------------------------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
	return ch;
 } 

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)ptr,len);	
	return len;
}
 
/*----------------------------------------------------------------------------*/
 
/*UART DMA Rx 副程式*/
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
	 if(huart==&huart1)
			{
				HAL_UART_Receive_DMA(huart,(uint8_t*)UART_Rx_buff,UART_Rx_BUFFSIZE);
			}
 }
/*---------------------------------------------------------------------------*/
 
/*UART DMA Printf 副程式*/
	int DMA_printf(UART_HandleTypeDef huart, const char * fmt,...)
{

	int printed;
	
	va_list args;
	
	va_start(args, fmt);

	printed = vsprintf((char *)Tx_buff,fmt,args);

	HAL_UART_Transmit_DMA(&huart,(uint8_t *)Tx_buff, printed);
	
	va_end(args);
	
	return printed;
}
/*---------------------------------------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET);
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	/*開啟中斷副程式*/
	
	/*4CH PWM Output*/
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); 	
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
 	/*RGB*/
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); 
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); 
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); 
	
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	/*DMA RX Enable*/
	HAL_UART_Receive_DMA(&huart1,(uint8_t*)UART_Rx_buff,UART_Rx_BUFFSIZE);
	/*EEPROM 初始化*/
	HAL_FLASH_Unlock();
	EE_Init();
	HAL_FLASH_Lock();
	
	/*初始化kalman*/
	Init(&kalmanX);
	Init(&kalmanY);

	/*馬達歸零*/
	TIM5->CCR1 = 1000;
	TIM5->CCR2 = 1000;
	TIM5->CCR3 = 1000;
	TIM5->CCR4 = 1000;
	HAL_Delay(5);
	RGB(1000,100,1000);
	/*MPU9250初始化*/
	MPU9250_Configuration(); 
	
	/*MPU9250 Gyro校正*/
	while(MPU9250_cal<3001) 
	{
		MPU9250_Gyro_calibr();	
	}
	HAL_Delay(1000);
//	/*磁力計隼位校正*/	
//	while(magcal<1001)
//	{
//		MPU9250_Magn_Levelcor();
//	}	
	/*MS5611初始化*/
	MS5611_Write(MS5611_CMD_Reset);
	HAL_Delay(10);
	MS5611_init();
	HAL_Delay(1000);
	RGB(600,300,1000);
	HAL_Delay(1000);
	RGB(100,1000,1000);
	/*載入 Angle PID Gain*/
	Load_Angle_PID_Gain();
	/*載入 Angular Velocity PID Gain*/
	Load_Angular_Velocity_PID_Gain();

	HAL_Delay(1000);
	RGB(1000,1000,100);

	HAL_Delay(1000);
	RGB(1000,100,1000);
	
  /* USER CODE END 2 */
	/*Test *********************************************/
  Gyro[0].angle = 0;
	Gyro[1].angle = 0;
	Gyro[2].angle = 0;	
	Magn[2].lastdata = 0;


	/***************************************************/
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  { 
		if(loop_100Hz_flag==5  )//100Hz
		{
			if(Fly_Start == 0)
			{
				RGB(200,1000,600);
				Main_meun();
			}
			//Magn_data();
			loop_100Hz_flag = 0;
		}
/*--------------------------------------------------------------------------------------------*/		
    if(loop_500Hz_flag==2)//500Hz
		{
			PID_Throttle.gain.p = 10;
			
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
			loop_500Hz_flag = 0;
			
			Gyro_data();
		  Accel_data();
			
			/*kalman filter*/		
			kalman_Roll		= getAngle(&kalmanX,Accel[1].angle,Gyro[1].new_raw,dt);
			kalman_Pitch	= getAngle(&kalmanY,Accel[0].angle,Gyro[0].new_raw,dt); 

			/*油門正規化與限制油門輸出*/
			throttle = Receiver_ideal_PWMMin + (((Receiver_ideal_PWMMax-Receiver_ideal_PWMMin)/(Receiver_PWMMax-Receiver_PWMMin))*(R12D[2]-Receiver_PWMMin));
			if(throttle > 1800 )throttle = 1800; 
/*--------------------------------------------------------------------------------------------*/				
			if(MS5611_Flag == 2)
			{
				//Read MS5611 Dt and TEMP
				MS5611.Dt = MS5611.D2 - MS5611.C[5]*256;
				MS5611.TEMP = 2000+((MS5611.Dt/100)*(MS5611.C[6]/100)/83886.08)*100;
				
				//MS5611溫度補償 
				if(MS5611.TEMP >=2000) 
				{
					MS5611.OFF2 = 0;
					MS5611.SENS2 = 0;
				}
				else if (MS5611.TEMP < 2000) 
				{
					MS5611.T2 = (MS5611.Dt*MS5611.Dt)/pow(2,31);
					MS5611.OFF2 = 5*(MS5611.TEMP-pow(2000,2))/2;
					MS5611.SENS2 = 5*pow((MS5611.TEMP-2000),2)/4;
				}
				//更新溫度資料
				MS5611.TEMP = MS5611.TEMP -MS5611.T2;
				MS5611.OFF = (((MS5611.C[2]/100)*655.35)*10000+(((MS5611.C[4]/100)*(MS5611.Dt/100))/1.28)*100)-MS5611.OFF2;
				MS5611.SENS = (((MS5611.C[1]/100)*327.68)*10000 +((MS5611.C[3]/100)*(MS5611.Dt/100)/2.56)*100)-MS5611.SENS2;
				MS5611.P = ((MS5611.D1 * MS5611.SENS/2097152)-MS5611.OFF)/32768;
				
				//平均濾波器
				MS5611_total_avarage -= MS5611_rotating_mem[MS5611_rotating_mem_location];
				MS5611_rotating_mem[MS5611_rotating_mem_location] = MS5611.P;
				MS5611_total_avarage += MS5611_rotating_mem[MS5611_rotating_mem_location]; 
				MS5611_rotating_mem_location++; 
				
				if (MS5611_rotating_mem_location == 20)MS5611_rotating_mem_location = 0;
				MS5611_actual_pressure_fast = (float)MS5611_total_avarage / 20.0; 
				MS5611_actual_pressure_slow = MS5611_actual_pressure_slow * (float)0.985 + MS5611_actual_pressure_fast * (float)0.015;
				MS5611_actual_pressure_diff = MS5611_actual_pressure_slow - MS5611_actual_pressure_fast;
				
				if (MS5611_actual_pressure_diff > 8)MS5611_actual_pressure_diff = 8;
				if (MS5611_actual_pressure_diff < -8)MS5611_actual_pressure_diff = -8;
				if (MS5611_actual_pressure_diff > 1 || MS5611_actual_pressure_diff < -1)MS5611_actual_pressure_slow -= MS5611_actual_pressure_diff / 6.0;
				
				MS5611_actual_pressure = MS5611_actual_pressure_slow;
				
				MS5611_Flag = 0; 
				MS5611_D1_COUNT = 0;
				MS5611_D2_COUNT = 0;
			}

/*--------------------------------------------------------------------------------------------*/	
			PID_Roll_Angle.FB  = kalman_Roll;
			PID_Pitch_Angle.FB = kalman_Pitch;
//			PID_Yaw_Angle.FB   = Gyro[2].angle;

			PID_Pitch.FB = Gyro[0].new_raw;
			PID_Roll.FB  = Gyro[1].new_raw;
			PID_Yaw.FB   = Gyro[2].new_raw;

 			PID_Throttle.FB = MS5611_actual_pressure;
			
			/*判斷解鎖條件*/
			if(R12D[2]<1100 && R12D[3]>1900 )
			{
				Fly_Start = 1;
				TIM5->CCR1=1000;
				TIM5->CCR2=1000;
				TIM5->CCR3=1000;
				TIM5->CCR4=1000;
				RGB(1000,1000,100);//藍色LED
			}
			/*確認搖桿回中*/
			if(Fly_Start == 1 && R12D[2]<1100 && R12D[3]<1550 )
			{
				Fly_flag=0;
				Fly_Start = 2;
				RGB(1000,100,100);//紫色LED
				
				/*PID-I Error 歸零*/
				/*Angular Velocity*/
				PID_Roll.error.i = 0;
				PID_Pitch.error.i = 0;
				PID_Yaw.error.i = 0;
				
				PID_Roll_Angle.error.i = 0;
				PID_Pitch_Angle.error.i = 0;
				PID_Yaw_Angle.error.i = 0;
				
				/*角度Lest_Error*/
				PID_Roll_Angle.lest_error.p = 0;
				PID_Pitch_Angle.lest_error.p = 0;
				PID_Yaw_Angle.lest_error.p = 0;			
				
				/*角速度Leat_Error*/
				PID_Roll.lest_error.p = 0;
				PID_Pitch.lest_error.p = 0;
				PID_Yaw.lest_error.p = 0;	
							
				/*角度Error D*/			
				PID_Pitch_Angle.error.d = Gyro[0].new_raw;		
				PID_Roll_Angle.error.d = Gyro[1].new_raw;
				
				Gyro[0].angle = 0;
				Gyro[1].angle = 0;
				Gyro[2].angle = 0;
			}
		  /*判斷上鎖條件*/
		  /*遙控器上鎖*/
		  if(Fly_Start==2 && R12D[2]<1100 && R12D[3]<1100)
		  {				
				Fly_Start = 0;
				TIM5->CCR1=1000;
				TIM5->CCR2=1000;
				TIM5->CCR3=1000;
				TIM5->CCR4=1000;
				RGB(1000,100,1000);//紅色LED
			}
		  /*超時上鎖*/
			if(Auto_attitude_flag==0)
			{
				if(Fly_Start==2 && R12D[2]<1100 && Fly_flag == 15000)
				{
					Fly_flag = 0;
					Fly_Start = 0;
					Gyro[2].angle = 0;
					TIM5->CCR1=1000;
					TIM5->CCR2=1000;
					TIM5->CCR3=1000;
					TIM5->CCR4=1000;
					RGB(1000,100,1000);//紅色LED
				}
			} 
/*--------------------------------------------------------------------------------------------*/	
			/*定高命令*/
			if(R12D[4]<1400)
			{
				PID_Throttle.Cmd = MS5611_actual_pressure;
			}
/*--------------------------------------------------------------------------------------------*/	
			/*PID 角度命令*/
			PID_Pitch_Angle.Cmd = 0;  
			if(R12D[1] > 1520)PID_Pitch_Angle.Cmd = ((R12D[1]-1520)*0.083);     
			else if(R12D[1] < 1480)PID_Pitch_Angle.Cmd = ((R12D[1]-1480)*0.083);

			PID_Roll_Angle.Cmd = 0;
			if(R12D[0] > 1520)PID_Roll_Angle.Cmd = ((R12D[0]-1520)*0.083);
			else if(R12D[0] < 1480)PID_Roll_Angle.Cmd = ((R12D[0]-1480)*0.083);

			PID_Yaw.Cmd = 0;   
			if(R12D[2] > 1200)
			{
				if(R12D[3] > 1520)PID_Yaw.Cmd = ((R12D[3]-1520)*0.083);
				else if(R12D[3] < 1480)PID_Yaw.Cmd = ((R12D[3]-1480)*0.083);
			}			
			/*三軸角度PID運算*/
			PID_Angle_controller(&PID_Roll_Angle);  
			PID_Angle_controller(&PID_Pitch_Angle);  
			//PID_Angle_controller(&PID_Yaw_Angle);
			
			/*PID 角速度命令*/
			PID_Pitch.Cmd = PID_Pitch_Angle.output; 
			PID_Roll.Cmd = PID_Roll_Angle.output;
			//PID_Yaw.Cmd = PID_Yaw_Angle.output;
			
			/*三軸角速度PID運算*/
			PID_controller(&PID_Roll);  
			PID_controller(&PID_Pitch);  
			PID_controller(&PID_Yaw);  
			
			/*高度PID*/
			
			PID_Throttle_controller(&PID_Throttle);
/*--------------------------------------------------------------------------------------------*/			
			if(Fly_Start == 2)
			{
				Fly_flag++;
				
				/*判斷是否進入定高模式*/
				if(R12D[4]>1600)
				{
					Auto_attitude_flag = 1;
					throttle = throttle + PID_Throttle.output;
				}
				else if(R12D[4]<1600)
				{
					Auto_attitude_flag = 0;
					throttle = throttle;
				}
				
				
				/*油門電壓補償*/
				if(Battery.Voltage <= 12.6f && Battery.Voltage > 11.1f)
				{
					throttle = throttle + (throttle*((11.1f - Battery.Voltage)/45.0f));
				}	
				else if (Battery.Voltage < 11.1f && Battery.Voltage > 8.0f)
				{
					throttle = throttle + (throttle*((11.1f - Battery.Voltage)/20.0f));
				}	
				
				/*馬達PWM輸出*/
				TIM5->CCR1 = (throttle - PID_Pitch.output + PID_Roll.output + PID_Yaw.output);
				TIM5->CCR2 = (throttle + PID_Pitch.output + PID_Roll.output - PID_Yaw.output);
				TIM5->CCR3 = (throttle + PID_Pitch.output - PID_Roll.output + PID_Yaw.output);
				TIM5->CCR4 = (throttle - PID_Pitch.output - PID_Roll.output - PID_Yaw.output);	

				/*設置PWM最大值與最小值*/
				if(TIM5->CCR1 <1100) TIM5->CCR1 = 1100; 	
				if(TIM5->CCR2 <1100) TIM5->CCR2 = 1100; 
				if(TIM5->CCR3 <1100) TIM5->CCR3 = 1100; 
				if(TIM5->CCR4 <1100) TIM5->CCR4 = 1100;      

				if(TIM5->CCR1 >2000) TIM5->CCR1 = 2000; 	
				if(TIM5->CCR2 >2000) TIM5->CCR2 = 2000; 
				if(TIM5->CCR3 >2000) TIM5->CCR3 = 2000; 
				if(TIM5->CCR4 >2000) TIM5->CCR4 = 2000; 
				/*數據*/
				//DMA_printf(huart1,"%8.3f\t %8.3f\t %8.3f\t %8.3f\n",kalman_Pitch,kalman_Roll,Gyro[2].new_raw,MS5611_actual_pressure);
				//DMA_printf(huart1,"%8.3f\t %8.3f\t %8.3f\n",kalman_Pitch,kalman_Roll,Gyro[2].new_raw);
				//DMA_printf(huart1,"%8.3f\t %8.3f\t %8.3f\n",PID_Pitch_Angle.Cmd,PID_Pitch_Angle.FB,PID_Pitch_Angle.output);
				//DMA_printf(huart1,"%8.3f\t %8.3f\t %8.3f\n",PID_Roll_Angle.Cmd,PID_Roll_Angle.FB,PID_Roll_Angle.output);
				//DMA_printf(huart1,"%8.3f\t %8.3f\t %8.3f\t %8.3f\t %8.3f\n",PID_Roll_Angle.Cmd,kalman_Roll,PID_Pitch_Angle.Cmd,kalman_Pitch,kalman_Yaw);
				//DMA_printf(huart1,"%8.1f\t %8.1f\t %8.1f\n",Gyro[1].angle,Accel[1].angle,kalman_Roll);
				//DMA_printf(huart1,"%8.1f\t %8.1f\t %8.1f\n",PID_Throttle.Cmd,PID_Throttle.FB,MS5611.TEMP);
				DMA_printf(huart1,"%8.1f\t %8.1f\t %8.1f\n",PID_Throttle.Cmd,PID_Throttle.FB,PID_Throttle.output);
			}
			else
			{
				TIM5->CCR1 = 1000; 	
				TIM5->CCR2 = 1000; 
				TIM5->CCR3 = 1000; 
				TIM5->CCR4 = 1000;  
			}
			/*電壓偵測*/
			if(ADC_Vaule()<10.0f)
			{
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
			}	
/*--------------------------------------------------------------------------------------------*/	
			/*MS5611 Read D1*/
			if(MS5611_Flag == 0)
			{ 
				if(MS5611_D1_COUNT == 0) MS5611_Write(MS5611_D1_CONV_OSR_4096); //Read D1
				
				if(MS5611_D1_COUNT == 4)
				{
					MS5611_D1_COUNT = 0;
					MS5611_Flag++;
					MS5611.D1 = MS5611_Read_24Bit(MS5611_CMD_AD_Read);
				}
				MS5611_D1_COUNT++; 
			}
			/*MS5611 Read D2*/
			if(MS5611_Flag == 1) 
			{
				if(MS5611_D2_COUNT == 0) MS5611_Write(MS5611_D2_CONV_OSR_4096); //Read D2
				if(MS5611_D2_COUNT == 4)
				{
					MS5611.D2 = MS5611_Read_24Bit(MS5611_CMD_AD_Read);
					MS5611_D2_COUNT = 0;
					MS5611_Flag++;
				}
				MS5611_D2_COUNT++;
			}
/*--------------------------------------------------------------------------------------------*/
		}	


  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
 }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
