#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <PID.h>
#include "eeprom.h"
#include "Battery.h"
#include "LED.h"
#include "attitude.h"

uint8_t First_menu=0,Second_menu=0,Third_menu = 0,YP_STATUS=0,YI_STATUS=0,YD_STATUS=0;

extern PID_TypeDef PID_Roll; 
extern PID_TypeDef PID_Pitch;
extern PID_TypeDef PID_Yaw;
extern PID_TypeDef PID_Roll_Angle;
extern PID_TypeDef PID_Pitch_Angle;
extern PID_TypeDef PID_Yaw_Angle;
extern PID_TypeDef PID_Throttle;
extern char UART_Rx_buff[3];
extern char *temrxdata;
extern uint8_t UART_Rx_COUNT,UART_Rx_BUFFSIZE;
extern ADC_battery Battery;
extern uint16_t R12D[12];
extern MPU9250 Magn[3];

int Get_menu_num(char* s)//選單副程式
{
	/*main menu--------------------*/
	if( !strcmp(s,"PTH"))return 1; //Pitch 
	if( !strcmp(s,"ROL"))return 2; //Roll
	if( !strcmp(s,"YAW"))return 3; //Yaw
	if( !strcmp(s,"BAT"))return 4; //Battery
	if( !strcmp(s,"HIG"))return 5; //Height
	/*Second menu------------------*/
	if( !strcmp(s,"AGE"))return 6;	//Angle
	if( !strcmp(s,"AGV"))return 7;	//Angular velocity
	if( !strcmp(s,"P++"))return 9;
	if( !strcmp(s,"P--"))return 10;
	if( !strcmp(s,"I++"))return 12;
	if( !strcmp(s,"I--"))return 13;
	if( !strcmp(s,"D++"))return 15;
	if( !strcmp(s,"D--"))return 16;
	/*Third menu-------------------*/
	if( !strcmp(s,"BAK"))return 17;	//BACK
	if( !strcmp(s,"YES"))return 18;	//YES
	if( !strcmp(s,"ACL"))return 14;	//Accel Calibration		
	if( !strcmp(s,"MCL"))return 11;	//Magnetometers Calibration
	
	/*NUL menu*********************/
	//if( !strcmp(s,""))return 8;
	
	return -1;
}

void Main_meun(void)
{
		switch(Get_menu_num(temrxdata))
    {
				case 1: //PTH
							First_menu = 1;
							//printf("The first_menu:PITCH\n");
							printf("The first_menu:PITCH\nPlease enter the AGE(angle) or AGV(angular velocity)\n");	 
							for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
							while(First_menu)//First_menu

							{
								Second_menu = 1;
								switch(Get_menu_num(temrxdata))
								{
									case 6://AGE
												//printf("The second menu:Angle\n PITCH Angle P-gain:%f\n PITCH Angle I-gain:%f\n PITCH Angle D-gain:%f\n",PID_Pitch_Angle.gain.p,PID_Pitch_Angle.gain.i,PID_Pitch_Angle.gain.d);
												printf("The second menu:Angle\n");
												printf("PITCH Angle P-gain:%f\n",PID_Pitch_Angle.gain.p);
												printf("PITCH Angle I-gain:%f\n",PID_Pitch_Angle.gain.i);
												printf("PITCH Angle D-gain:%f\n",PID_Pitch_Angle.gain.d);
									
									
												for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}	
												while(Second_menu)
												{	
													switch(Get_menu_num(temrxdata))//second_menu
													{
														case 9://P++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch_Angle.gain.p = PID_Pitch_Angle.gain.p + 0.1f;
																	PID_Pitch_Angle.Buff.p = PID_Pitch_Angle.gain.p*1000;
																	printf("Angle P-Gain:%f\n",PID_Pitch_Angle.gain.p);
																	YP_STATUS = 1;
																	break;				
														case 10://P--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch_Angle.gain.p = PID_Pitch_Angle.gain.p - 0.1f;
																	PID_Pitch_Angle.Buff.p = PID_Pitch_Angle.gain.p*1000;
																	printf("Angle P-Gain:%f\n",PID_Pitch_Angle.gain.p);
																	YP_STATUS = 1;
																	break;
													 case 12://I++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch_Angle.gain.i = PID_Pitch_Angle.gain.i + 0.01f;
																	PID_Pitch_Angle.Buff.i = PID_Pitch_Angle.gain.i*1000;
																	printf("Angle I-Gain:%f\n",PID_Pitch_Angle.gain.i);
																	YI_STATUS = 1;
																	break;
														case 13://I--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch_Angle.gain.i = PID_Pitch_Angle.gain.i - 0.01f;
																	PID_Pitch_Angle.Buff.i = PID_Pitch_Angle.gain.i*1000;
																	printf("Angle I-Gain:%f\n",PID_Pitch_Angle.gain.i);
																	YI_STATUS = 1;
																	break;
														case 15://D++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch_Angle.gain.d = PID_Pitch_Angle.gain.d + 0.1f;
																	PID_Pitch_Angle.Buff.d = PID_Pitch_Angle.gain.d*1000;
																	printf("Angle D-Gain:%f\n",PID_Pitch_Angle.gain.d);
																	YD_STATUS = 1;
																	break;
														case 16://D--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch_Angle.gain.d = PID_Pitch_Angle.gain.d - 0.1f;
																	PID_Pitch_Angle.Buff.d = PID_Pitch_Angle.gain.d*1000;
																	printf("Angle D-Gain:%f\n",PID_Pitch_Angle.gain.d);
																	YD_STATUS = 1;
																	break;
														case 17://BAK
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	Second_menu = 0;
																	printf("Please enter the AGE(angle) or AGV(angular velocity)\n");		 
																	break;
														case 18://YES
																	if(YP_STATUS>0 || YI_STATUS>0  || YD_STATUS>0 )
																	{
																		if(YP_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angle_Pitch_P_EE_Address,PID_Pitch_Angle.Buff.p);
																			HAL_FLASH_Lock();
																			printf("PITCH Angle P-Gain has been revised successfully: %f\n",PID_Pitch_Angle.Buff.p*0.001);
																			YP_STATUS = 0;
																		}
																		else if(YI_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angle_Pitch_I_EE_Address,PID_Pitch_Angle.Buff.i);
																			HAL_FLASH_Lock();
																			printf("PITCH Angle I-Gain has been revised successfully: %f\n",PID_Pitch_Angle.Buff.i*0.001);
																			YI_STATUS = 0;
																		}
																		else if(YD_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angle_Pitch_D_EE_Address,PID_Pitch_Angle.Buff.d);
																			HAL_FLASH_Lock();
																			printf("PITCH Angle D-Gain has been revised successfully: %f\n",PID_Pitch_Angle.Buff.d*0.001);
																			YD_STATUS = 0;
																		}											
																	}
																	else if(YP_STATUS==0 && YI_STATUS==0  && YD_STATUS==0 )
																	{
																		for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																		printf("OK!\n");
																	}
																	break;																																																															
													}
												}
												break;

									case 7://AGV	
												printf("Now is the second menu:Angular velocity\n");
												printf("PITCH Angular velocity P-gain:%f\n",PID_Pitch.gain.p);
												printf("PITCH Angular velocity I-gain:%f\n",PID_Pitch.gain.i);
												printf("PITCH Angular velocity D-gain:%f\n",PID_Pitch.gain.d);
												for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}	
												while(Second_menu)
                        {
													switch(Get_menu_num(temrxdata))//second_menu
													{
														case 9://P++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch.gain.p = PID_Pitch.gain.p + 0.1f;
																	PID_Pitch.Buff.p = PID_Pitch.gain.p*1000;
																	printf("Angular_Velocity P-Gain:%f\n",PID_Pitch.gain.p);
																	YP_STATUS = 1;
																	break;				
														case 10://P--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch.gain.p = PID_Pitch.gain.p - 0.1f;
																	PID_Pitch.Buff.p = PID_Pitch.gain.p*1000;
																	printf("Angular_Velocity P-Gain:%f\n",PID_Pitch.gain.p);
																	YP_STATUS = 1;
																	break;
													  case 12://I++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch.gain.i = PID_Pitch.gain.i + 0.01f;
																	PID_Pitch.Buff.i = PID_Pitch.gain.i*1000;
																	printf("Angular_Velocity I-Gain:%f\n",PID_Pitch.gain.i);
																	YI_STATUS = 1;
																	break;
														case 13://I--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch.gain.i = PID_Pitch.gain.i - 0.01f;
																	PID_Pitch.Buff.i=PID_Pitch.gain.i*1000;
																	printf("Angular_Velocity I-Gain:%f\n",PID_Pitch.gain.i);
																	YI_STATUS = 1;
																	break;
														case 15://D++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch.gain.d = PID_Pitch.gain.d + 0.1f;
																	PID_Pitch.Buff.d = PID_Pitch.gain.d*1000;
																	printf("Angular_Velocity D-Gain:%f\n",PID_Pitch.gain.d);
																	YD_STATUS = 1;
																	break;
														case 16://D--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Pitch.gain.d = PID_Pitch.gain.d - 0.1f;
																	PID_Pitch.Buff.d = PID_Pitch.gain.d*1000;
																	printf("Angular_Velocity D-Gain:%f\n",PID_Pitch.gain.d);
																	YD_STATUS = 1;
																	break;
														case 17://BAK
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	Second_menu = 0;
																	printf("Please enter the AGE(angle) or AGV(angular velocity)\n");		 
																	break;
														case 18://YES
																	if(YP_STATUS>0 || YI_STATUS>0  || YD_STATUS>0 )
																	{
																		if(YP_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angular_Velocity_Pitch_P_EE_Address,PID_Pitch.Buff.p);
																			HAL_FLASH_Lock();
																			printf("PITCH Angular_Velocity P-Gain has been revised successfully: %f\n",PID_Pitch.Buff.p*0.001);
																			YP_STATUS = 0;
																		}
																		else if(YI_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angular_Velocity_Pitch_I_EE_Address,PID_Pitch.Buff.i);
																			HAL_FLASH_Lock();
																			printf("PITCH Angular_Velocity I-Gain has been revised successfully: %f\n",PID_Pitch.Buff.i*0.001);
																			YI_STATUS = 0;
																		}
																		else if(YD_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angular_Velocity_Pitch_D_EE_Address,PID_Pitch.Buff.d);
																			HAL_FLASH_Lock();
																			printf("PITCH Angular_Velocity D-Gain has been revised successfully: %f\n",PID_Pitch.Buff.d*0.001);
																			YD_STATUS = 0;
																		}											
																	}
																	else if(YP_STATUS==0 && YI_STATUS==0  && YD_STATUS==0 )
																	{
																		for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																		printf("OK!\n");
																	}
																	break;																																																														
													}
												}
											  break;
												
									case 17://BAK
												for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
												printf("Please enter the command\n");
												First_menu = 0;
												break;	
								}
							}
							break;
/*----------------------------------------------------------------------------------------------------*/
				case 2: //ROL
							First_menu = 1;
							printf("\nThe first_menu:ROLL\n");
							printf("\nPlease enter the AGE(angle) or AGV(angular velocity)\n");	 
							for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
							while(First_menu)//First_menu
							{
								Second_menu = 1;
								switch(Get_menu_num(temrxdata))
								{
									case 6://AGE
												printf("\nThe second menu:Angle\n");
												printf("ROLL Angle P-gain:%f\n",PID_Roll_Angle.gain.p);
												printf("ROLL Angle I-gain:%f\n",PID_Roll_Angle.gain.i);
												printf("ROLL Angle D-gain:%f\n",PID_Roll_Angle.gain.d);
												for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}	
												while(Second_menu)
												{	
													switch(Get_menu_num(temrxdata))//second_menu
													{
														case 9://P++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll_Angle.gain.p = PID_Roll_Angle.gain.p + 0.1f;
																	PID_Roll_Angle.Buff.p = PID_Roll_Angle.gain.p*1000;
																	printf("Angle P-Gain:%f\n",PID_Roll_Angle.gain.p);
																	YP_STATUS = 1;
																	break;				
														case 10://P--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll_Angle.gain.p = PID_Roll_Angle.gain.p - 0.1f;
																	PID_Roll_Angle.Buff.p = PID_Roll_Angle.gain.p*1000;
																	printf("Angle P-Gain:%f\n",PID_Roll_Angle.gain.p);
																	YP_STATUS = 1;
																	break;
													 case 12://I++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll_Angle.gain.i = PID_Roll_Angle.gain.i + 0.01f;
																	PID_Roll_Angle.Buff.i = PID_Roll_Angle.gain.i*1000;
																	printf("Angle I-Gain:%f\n",PID_Roll_Angle.gain.i);
																	YI_STATUS = 1;
																	break;
														case 13://I--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll_Angle.gain.i = PID_Roll_Angle.gain.i - 0.01f;
																	PID_Roll_Angle.Buff.i = PID_Roll_Angle.gain.i*1000;
																	printf("Angle I-Gain:%f\n",PID_Roll_Angle.gain.i);
																	YI_STATUS = 1;
																	break;
														case 15://D++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll_Angle.gain.d = PID_Roll_Angle.gain.d + 0.1f;
																	PID_Roll_Angle.Buff.d = PID_Roll_Angle.gain.d*1000;
																	printf("Angle D-Gain:%f\n",PID_Roll_Angle.gain.d);
																	YD_STATUS = 1;
																	break;
														case 16://D--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll_Angle.gain.d = PID_Roll_Angle.gain.d - 0.1f;
																	PID_Roll_Angle.Buff.d = PID_Roll_Angle.gain.d*1000;
																	printf("Angle D-Gain:%f\n",PID_Roll_Angle.gain.d);
																	YD_STATUS = 1;
																	break;
														case 17://BAK
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	Second_menu = 0;
																	printf("\nPlease enter the AGE(angle) or AGV(angular velocity)\n");		 
																	break;
														case 18://YES
																	if(YP_STATUS>0 || YI_STATUS>0  || YD_STATUS>0 )
																	{
																		if(YP_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angle_Roll_P_EE_Address,PID_Roll_Angle.Buff.p);
																			HAL_FLASH_Lock();
																			printf("\nROLL Angle P-Gain has been revised successfully: %f\n",PID_Roll_Angle.Buff.p*0.001);
																			YP_STATUS = 0;
																		}
																		else if(YI_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angle_Roll_I_EE_Address,PID_Roll_Angle.Buff.i);
																			HAL_FLASH_Lock();
																			printf("\nROLL Angle I-Gain has been revised successfully: %f\n",PID_Roll_Angle.Buff.i*0.001);
																			YI_STATUS = 0;
																		}
																		else if(YD_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angle_Roll_D_EE_Address,PID_Roll_Angle.Buff.d);
																			HAL_FLASH_Lock();
																			printf("\nROLL Angle D-Gain has been revised successfully: %f\n",PID_Roll_Angle.Buff.d*0.001);
																			YD_STATUS = 0;
																		}											
																	}
																	else if(YP_STATUS==0 && YI_STATUS==0  && YD_STATUS==0 )
																	{
																		for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																		printf("\nOK!\n");
																	}
																	break;																																																															
													}
												}
												break;
			
									case 7://AGV	
												printf("\nNow is the second menu:Angular velocity\n");
												printf("\nROLL Angular velocity P-gain:%f\n",PID_Roll.gain.p);
												printf("\nROLL Angular velocity I-gain:%f\n",PID_Roll.gain.i);
												printf("\nROLL Angular velocity D-gain:%f\n",PID_Roll.gain.d);
												for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}	
												while(Second_menu)
                        {
													switch(Get_menu_num(temrxdata))//second_menu
													{
														case 9://P++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll.gain.p = PID_Roll.gain.p + 0.1f;
																	PID_Roll.Buff.p = PID_Roll.gain.p*1000;
																	printf("Angular_Velocity P-Gain:%f\n",PID_Roll.gain.p);
																	YP_STATUS = 1;
																	break;				
														case 10://P--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll.gain.p = PID_Roll.gain.p - 0.1f;
																	PID_Roll.Buff.p = PID_Roll.gain.p*1000;
																	printf("Angular_Velocity P-Gain:%f\n",PID_Roll.gain.p);
																	YP_STATUS = 1;
																	break;
													  case 12://I++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll.gain.i = PID_Roll.gain.i + 0.01f;
																	PID_Roll.Buff.i = PID_Roll.gain.i*1000;
																	printf("Angular_Velocity I-Gain:%f\n",PID_Roll.gain.i);
																	YI_STATUS = 1;
																	break;
														case 13://I--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll.gain.i = PID_Roll.gain.i - 0.01f;
																	PID_Roll.Buff.i=PID_Roll.gain.i*1000;
																	printf("Angular_Velocity I-Gain:%f\n",PID_Roll.gain.i);
																	YI_STATUS = 1;
																	break;
														case 15://D++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll.gain.d = PID_Roll.gain.d + 0.1f;
																	PID_Roll.Buff.d = PID_Roll.gain.d*1000;
																	printf("Angular_Velocity D-Gain:%f\n",PID_Roll.gain.d);
																	YD_STATUS = 1;
																	break;
														case 16://D--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Roll.gain.d = PID_Roll.gain.d - 0.1f;
																	PID_Roll.Buff.d = PID_Roll.gain.d*1000;
																	printf("Angular_Velocity D-Gain:%f\n",PID_Roll.gain.d);
																	YD_STATUS = 1;
																	break;
														case 17://BAK
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	Second_menu = 0;
																	printf("\nPlease enter the AGE(angle) or AGV(angular velocity)\n");		 
																	break;
														case 18://YES
																	if(YP_STATUS>0 || YI_STATUS>0  || YD_STATUS>0 )
																	{
																		if(YP_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angular_Velocity_Roll_P_EE_Address,PID_Roll.Buff.p);
																			HAL_FLASH_Lock();
																			printf("\nROLL Angular_Velocity P-Gain has been revised successfully: %f\n",PID_Roll.Buff.p*0.001);
																			YP_STATUS = 0;
																		}
																		else if(YI_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angular_Velocity_Roll_I_EE_Address,PID_Roll.Buff.i);
																			HAL_FLASH_Lock();
																			printf("\nROLL Angular_Velocity I-Gain has been revised successfully: %f\n",PID_Roll.Buff.i*0.001);
																			YI_STATUS = 0;
																		}
																		else if(YD_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angular_Velocity_Roll_D_EE_Address,PID_Roll.Buff.d);
																			HAL_FLASH_Lock();
																			printf("\nROLL Angular_Velocity D-Gain has been revised successfully: %f\n",PID_Roll.Buff.d*0.001);
																			YD_STATUS = 0;
																		}											
																	}
																	else if(YP_STATUS==0 && YI_STATUS==0  && YD_STATUS==0 )
																	{
																		for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																		printf("\nOK!\n");
																	}
																	break;																																																														
													}
												}
											  break;
									case 17://BAK
												for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
												printf("\nPlease enter the command\n");
												First_menu = 0;
												break;													
								}
							}
							break;
/*----------------------------------------------------------------------------------------------------*/
				case 3: //YAW
							First_menu = 1;
							printf("\nThe first_menu:YAW\n");
							printf("\nPlease enter the AGE(angle) or AGV(angular velocity)\n");	 
							for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
							while(First_menu)//First_menu

							{
								Second_menu = 1;
								switch(Get_menu_num(temrxdata))
								{
									case 6://AGE
												printf("\nThe second menu:Angle\n");
												printf("YAW Angle P-gain:%f\n",PID_Yaw_Angle.gain.p);
												printf("YAW Angle I-gain:%f\n",PID_Yaw_Angle.gain.i);
												printf("YAW Angle D-gain:%f\n",PID_Yaw_Angle.gain.d);
												for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}	
												while(Second_menu)
												{	
													switch(Get_menu_num(temrxdata))//second_menu
													{
														case 9://P++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw_Angle.gain.p = PID_Yaw_Angle.gain.p + 0.1f;
																	PID_Yaw_Angle.Buff.p = PID_Yaw_Angle.gain.p*1000;
																	printf("Angle P-Gain:%f\n",PID_Yaw_Angle.gain.p);
																	YP_STATUS = 1;
																	break;				
														case 10://P--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw_Angle.gain.p = PID_Yaw_Angle.gain.p - 0.1f;
																	PID_Yaw_Angle.Buff.p = PID_Yaw_Angle.gain.p*1000;
																	printf("Angle P-Gain:%f\n",PID_Yaw_Angle.gain.p);
																	YP_STATUS = 1;
																	break;
													 case 12://I++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw_Angle.gain.i = PID_Yaw_Angle.gain.i + 0.01f;
																	PID_Yaw_Angle.Buff.i = PID_Yaw_Angle.gain.i*1000;
																	printf("Angle I-Gain:%f\n",PID_Yaw_Angle.gain.i);
																	YI_STATUS = 1;
																	break;
														case 13://I--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw_Angle.gain.i = PID_Yaw_Angle.gain.i - 0.01f;
																	PID_Yaw_Angle.Buff.i = PID_Yaw_Angle.gain.i*1000;
																	printf("Angle I-Gain:%f\n",PID_Yaw_Angle.gain.i);
																	YI_STATUS = 1;
																	break;
														case 15://D++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw_Angle.gain.d = PID_Yaw_Angle.gain.d + 0.1f;
																	PID_Yaw_Angle.Buff.d = PID_Yaw_Angle.gain.d*1000;
																	printf("Angle D-Gain:%f\n",PID_Yaw_Angle.gain.d);
																	YD_STATUS = 1;
																	break;
														case 16://D--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw_Angle.gain.d = PID_Yaw_Angle.gain.d - 0.1f;
																	PID_Yaw_Angle.Buff.d = PID_Yaw_Angle.gain.d*1000;
																	printf("Angle D-Gain:%f\n",PID_Yaw_Angle.gain.d);
																	YD_STATUS = 1;
																	break;
														case 17://BAK
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	Second_menu = 0;
																	printf("\nPlease enter the AGE(angle) or AGV(angular velocity)\n");		 
																	break;
														case 18://YES
																	if(YP_STATUS>0 || YI_STATUS>0  || YD_STATUS>0 )
																	{
																		if(YP_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angle_Yaw_P_EE_Address,PID_Yaw_Angle.Buff.p);
																			HAL_FLASH_Lock();
																			printf("\nYAW Angle P-Gain has been revised successfully: %f\n",PID_Yaw_Angle.Buff.p*0.001);
																			YP_STATUS = 0;
																		}
																		else if(YI_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angle_Yaw_I_EE_Address,PID_Yaw_Angle.Buff.i);
																			HAL_FLASH_Lock();
																			printf("\nYAW Angle I-Gain has been revised successfully: %f\n",PID_Yaw_Angle.Buff.i*0.001);
																			YI_STATUS = 0;
																		}
																		else if(YD_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angle_Yaw_D_EE_Address,PID_Yaw_Angle.Buff.d);
																			HAL_FLASH_Lock();
																			printf("\nYAW Angle D-Gain has been revised successfully: %f\n",PID_Yaw_Angle.Buff.d*0.001);
																			YD_STATUS = 0;
																		}											
																	}
																	else if(YP_STATUS==0 && YI_STATUS==0  && YD_STATUS==0 )
																	{
																		for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																		printf("\nOK!\n");
																	}
																	break;																																																															
													}
												}
												break;
			
									case 7://AGV	
												printf("\nNow is the second menu:Angular velocity\n");
												printf("\nYAW Angular velocity P-gain:%f\n",PID_Yaw.gain.p);
												printf("\nYAW Angular velocity I-gain:%f\n",PID_Yaw.gain.i);
												printf("\nYAW Angular velocity D-gain:%f\n",PID_Yaw.gain.d);
												for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}	
												while(Second_menu)
                        {
													switch(Get_menu_num(temrxdata))//second_menu
													{
														case 9://P++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw.gain.p = PID_Yaw.gain.p + 0.1f;
																	PID_Yaw.Buff.p = PID_Yaw.gain.p*1000;
																	printf("Angular_Velocity P-Gain:%f\n",PID_Yaw.gain.p);
																	YP_STATUS = 1;
																	break;				
														case 10://P--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw.gain.p = PID_Yaw.gain.p - 0.1f;
																	PID_Yaw.Buff.p = PID_Yaw.gain.p*1000;
																	printf("Angular_Velocity P-Gain:%f\n",PID_Yaw.gain.p);
																	YP_STATUS = 1;
																	break;
													  case 12://I++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw.gain.i = PID_Yaw.gain.i + 0.01f;
																	PID_Yaw.Buff.i = PID_Yaw.gain.i*1000;
																	printf("Angular_Velocity I-Gain:%f\n",PID_Yaw.gain.i);
																	YI_STATUS = 1;
																	break;
														case 13://I--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw.gain.i = PID_Yaw.gain.i - 0.01f;
																	PID_Yaw.Buff.i=PID_Yaw.gain.i*1000;
																	printf("Angular_Velocity I-Gain:%f\n",PID_Yaw.gain.i);
																	YI_STATUS = 1;
																	break;
														case 15://D++
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw.gain.d = PID_Yaw.gain.d + 0.1f;
																	PID_Yaw.Buff.d = PID_Yaw.gain.d*1000;
																	printf("Angular_Velocity D-Gain:%f\n",PID_Yaw.gain.d);
																	YD_STATUS = 1;
																	break;
														case 16://D--
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	PID_Yaw.gain.d = PID_Yaw.gain.d - 0.1f;
																	PID_Yaw.Buff.d = PID_Yaw.gain.d*1000;
																	printf("Angular_Velocity D-Gain:%f\n",PID_Yaw.gain.d);
																	YD_STATUS = 1;
																	break;
														case 17://BAK
																	for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																	Second_menu = 0;
																	printf("\nPlease enter the AGE(angle) or AGV(angular velocity)\n");		 
																	break;
														case 18://YES
																	if(YP_STATUS>0 || YI_STATUS>0  || YD_STATUS>0 )
																	{
																		if(YP_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angular_Velocity_Yaw_P_EE_Address,PID_Yaw.Buff.p);
																			HAL_FLASH_Lock();
																			printf("\nYAW Angular_Velocity P-Gain has been revised successfully: %f\n",PID_Yaw.Buff.p*0.001);
																			YP_STATUS = 0;
																		}
																		else if(YI_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angular_Velocity_Yaw_I_EE_Address,PID_Yaw.Buff.i);
																			HAL_FLASH_Lock();
																			printf("\nYAW Angular_Velocity I-Gain has been revised successfully: %f\n",PID_Yaw.Buff.i*0.001);
																			YI_STATUS = 0;
																		}
																		else if(YD_STATUS==1)
																		{
																			HAL_FLASH_Unlock();
																			EE_Init();
																			EE_WriteVariable(PID_Angular_Velocity_Yaw_D_EE_Address,PID_Yaw.Buff.d);
																			HAL_FLASH_Lock();
																			printf("\nYAW Angular_Velocity D-Gain has been revised successfully: %f\n",PID_Yaw.Buff.d*0.001);
																			YD_STATUS = 0;
																		}											
																	}
																	else if(YP_STATUS==0 && YI_STATUS==0  && YD_STATUS==0 )
																	{
																		for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
																		printf("\nOK!\n");
																	}
																	break;																																																														
													}
												}
											  break;
									case 17://BAK
												for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
												printf("\nPlease enter the command\n");
												First_menu = 0;
												break;													
								}
							}
							break;
/*----------------------------------------------------------------------------------------------------*/
			 case 4:	//BAT
						 printf("\nBattery Voltage: %4.2f\n",Battery.Voltage);
						 for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
						 break;
/*----------------------------------------------------------------------------------------------------*/
			 case 5:	//HIG
							Second_menu = 1;
				 			printf("\nThe first_menu:PITCH\n");
							printf("Height P-gain:%f\n",PID_Throttle.gain.p);
							printf("Height I-gain:%f\n",PID_Throttle.gain.i);
							printf("Height D-gain:%f\n",PID_Throttle.gain.d);			
							
						 for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}		
						 while(Second_menu)
						 {	
							switch(Get_menu_num(temrxdata))//second_menu
							{
								case 9://P++
											for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
											
											PID_Throttle.gain.p = PID_Throttle.gain.p + 0.1f;
											PID_Throttle.Buff.p = PID_Throttle.gain.p*1000;
											printf("Height P-Gain:%f\n",PID_Throttle.gain.p);
											break;				
								case 10://P--
											for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
											PID_Throttle.gain.p = PID_Throttle.gain.p - 0.1f;
											PID_Throttle.Buff.p = PID_Throttle.gain.p*1000;
											printf("Height P-Gain:%f\n",PID_Throttle.gain.p);
											break;
							 case 12://I++
											for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
											PID_Throttle.gain.i = PID_Throttle.gain.i + 0.01f;
											PID_Throttle.Buff.i = PID_Throttle.gain.i*1000;
											printf("Height I-Gain:%f\n",PID_Throttle.gain.i);
											break;
								case 13://I--
											for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
											PID_Throttle.gain.i = PID_Throttle.gain.i - 0.01f;
											PID_Throttle.Buff.i = PID_Throttle.gain.i*1000;
											printf("Height I-Gain:%f\n",PID_Throttle.gain.i);
											break;
								case 15://D++
											for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
											PID_Throttle.gain.d = PID_Throttle.gain.d + 0.1f;
											PID_Throttle.Buff.d = PID_Throttle.gain.d*1000;
											printf("Height D-Gain:%f\n",PID_Throttle.gain.d);
											break;
								case 16://D--
											for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
											PID_Throttle.gain.d = PID_Throttle.gain.d - 0.1f;
											PID_Throttle.Buff.d = PID_Throttle.gain.d*1000;
											printf("Height D-Gain:%f\n",PID_Throttle.gain.d);
											break;
								case 17://BAK
											for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
											Second_menu = 0;
											printf("\nPlease enter the command\n");		 
											break;
										}
									}
						 break;
/*----------------------------------------------------------------------------------------------------*/
			 case 14:	//ACL
							printf("\nREADY to correct the Accel offset value\n");
							if(R12D[8]>1600 || R12D[9]>1600)
							{
								printf("\nError!! Please turn ON Receiver_accleCar_check and Receiver_accleCar_Lock\n");
							}
							else if(R12D[8]<1600 && R12D[9]<1600)
							{
								First_menu = 1;
								printf("\nReceiver_accelCar_check:OFF  Receiver_accelCar_Lock:OFF");
								while(First_menu)//First_menu
								{
									MPU9250_Accle_calibr();
								}
							}
							for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
							break;
/*----------------------------------------------------------------------------------------------------*/	
			 case 11:	//MCL
						  printf("\nREADY to correct the Magnetometers offset value\n");
							if(R12D[8]>1600 || R12D[9]>1600)
							{
								printf("\nError!! Please turn ON Receiver_MagnetCar_check and Receiver_MagnetCar_Lock\n");
							}
							else if(R12D[8]<1600 && R12D[9]<1600)
							{
								First_menu = 1;
								printf("\nReceiver_MagnetCar_check:OFF  Receiver_MagnetCar_Lock:OFF");
								while(First_menu)//First_menu
								{
									MPU9250_Magn_calibr();
								}
								printf("\n X-offset:%g\t Y-offset:%g\t Z-offset:%g\t X-Cali:%g\t Y-Cali:%g\t Z-Cali:%g\n",Magn[0].offset,Magn[1].offset,Magn[2].offset,Magn[0].Calibration,Magn[1].Calibration,Magn[2].Calibration);
							}
							for(int a=0; a<=UART_Rx_BUFFSIZE ;a++){UART_Rx_buff[a] = 0;}
							break;				
							
							
							
							
			
							
		}		
		RGB(1000,100,1000);
}

