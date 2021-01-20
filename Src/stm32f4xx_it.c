/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
/* USER CODE BEGIN 0 */
#include "Kalman.h"
#include "MPU9250.h"
#include "attitude.h"
#include "Battery.h"
#include "PID.h"
#include "LED.h"

/*»»±±¾¹ÅÜ¼Æ*/
uint16_t last_receiver[12];
uint16_t receiver[12];
uint16_t R12D[12];
uint8_t starus = 0,i=0;
	
extern int8_t loop_10Hz_flag,loop_500Hz_flag,loop_100Hz_flag,loop_50Hz_flag;
extern int16_t Fly_flag; 
extern uint8_t ACL_flag;
extern sensor IMU;
extern MPU9250 Gyro[3];
extern MPU9250 Accel[3];
extern MPU9250 Magn[3];
extern float magn_zeroPoint;

extern struct Kalman kalmanX,kalmanY,kalmanZ; 
extern float kalman_Pitch,kalman_Roll,kalman_Yaw;   

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	
  

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	
	
	
	

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
  //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 0) 
	 {
		starus = 1;
	  last_receiver[0] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 1) 
	 {
		 starus = 2;
	   receiver[0] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[0] = receiver[0]-last_receiver[0];
		 last_receiver[0] = __HAL_TIM_SetCounter(&htim3,0);
		 if(R12D[0] >3800)
		 {
			 starus = 22;
			 
		 }
	 }
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 2) 
	 {
		starus = 3;
	  last_receiver[1] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 3) 
	 {
		 starus = 4;
	   receiver[1] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[1] = receiver[1]-last_receiver[1];
		 last_receiver[1] = __HAL_TIM_SetCounter(&htim3,0);
		 
		 if(R12D[1] >3800)
		 {
			 starus = 22;
		 }
	 }
	 
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 4) 
	 {
		starus = 5;
	  last_receiver[2] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 5) 
	 {
		 starus = 6;
	   receiver[2] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[2] = receiver[2]-last_receiver[2];
		 last_receiver[2] = __HAL_TIM_SetCounter(&htim3,0);
		 
		 if(R12D[2] >3800)
		 {
		
			 starus = 22;
		 }
	 }
	 
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 6) 
	 {
		starus = 7;
	  last_receiver[3] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 7) 
	 {
		 starus = 8;
	   receiver[3] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[3] = receiver[3]-last_receiver[3];
		 last_receiver[3] = __HAL_TIM_SetCounter(&htim3,0);
		 
		 if(R12D[3] >3800)
		 {
			
			 starus = 22;
		 }
	 }
	 
	
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 8) 
	 {
		starus = 9;
	  last_receiver[4] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 9) 
	 {
		 starus = 10;
	   receiver[4] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[4] = receiver[4]-last_receiver[4];
		 last_receiver[4] = __HAL_TIM_SetCounter(&htim3,0);
		 
		 if(R12D[4] >3800)
		 {
			 
			 starus = 22;
		 }
	 }
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 10) 
	 {
		starus = 11;
	  last_receiver[5] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 11) 
	 {
		 starus = 12;
	   receiver[5] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[5] = receiver[5]-last_receiver[5];
		 last_receiver[5] = __HAL_TIM_SetCounter(&htim3,0);
		 
		 if(R12D[5] >3800)
		 {
			
			 starus = 22;
		 }
	 }
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 12) 
	 {
		starus = 13;
	  last_receiver[6] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 13) 
	 {
		 starus = 14;
	   receiver[6] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[6] = receiver[6]-last_receiver[6];
		 last_receiver[6] = __HAL_TIM_SetCounter(&htim3,0);
		 
		 if(R12D[6] >3800)
		 {
			
			 starus = 22;
		 }
	 }
	 
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 14) 
	 {
		starus = 15;
	  last_receiver[7] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 15) 
	 {
		 starus = 16;
	   receiver[7] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[7] = receiver[7]-last_receiver[7];
		 last_receiver[7] = __HAL_TIM_SetCounter(&htim3,0);
		 
		 if(R12D[7] >3800)
		 {
			
			 starus = 22;
		 }
	 }
	 
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 16) 
	 {
		starus = 17;
	  last_receiver[8] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 17) 
	 {
		 starus = 18;
	   receiver[8] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[8] = receiver[8]-last_receiver[8];
		 last_receiver[8] = __HAL_TIM_SetCounter(&htim3,0);
		 
		 if(R12D[8] >3800)
		 {
			 
			 starus = 22;
		 }
	 }
	 
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 18) 
	 {
		starus = 19;
	  last_receiver[9] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 19) 
	 {
		 starus = 20;
	   receiver[9] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[9] = receiver[9]-last_receiver[9];
		 last_receiver[9] = __HAL_TIM_SetCounter(&htim3,0);
		 
		 if(R12D[9] >3800)
		 {
		
			 starus = 22;
		 }
	 }
	 
	 
	 if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 20) 
	 {
		starus = 21;
	  last_receiver[10] = __HAL_TIM_GetCounter(&htim3);
	 }
	 else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) != RESET && starus == 21) 
	 {
		 starus = 22;
	   receiver[10] = __HAL_TIM_GetCounter(&htim3); 
		 R12D[10] = receiver[10]-last_receiver[10];
		 last_receiver[10] = __HAL_TIM_SetCounter(&htim3,0); 
	 }
	 
	 if(starus == 22)starus = 1;
	
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)

{
  /* USER CODE BEGIN TIM3_IRQn 0 */

	
	
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
  /* USER CODE BEGIN TIM4_IRQn 0 */

	
	Gyro_data();
	Accel_data();
//	/*kalman filter*/		
//	kalman_Pitch	= getAngle(&kalmanY,Accel[0].angle,Gyro[0].new_raw,0.002);
//	kalman_Roll		= getAngle(&kalmanX,Accel[1].angle,Gyro[1].new_raw,0.002);

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  loop_500Hz_flag++;
	loop_100Hz_flag++;
	loop_50Hz_flag++;
	loop_10Hz_flag++;
	ACL_flag++;
  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles TIM8 capture compare interrupt.
*/
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */

  /* USER CODE END TIM8_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream7 global interrupt.
*/
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
