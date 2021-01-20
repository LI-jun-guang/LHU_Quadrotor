#include "stm32f4xx_hal.h"
#include "PID.h"
#include "eeprom.h"

PID_TypeDef PID_Roll; 
PID_TypeDef PID_Pitch;
PID_TypeDef PID_Yaw;

PID_TypeDef PID_Roll_Angle;
PID_TypeDef PID_Pitch_Angle;
PID_TypeDef PID_Yaw_Angle;

PID_TypeDef PID_Throttle;

/*載入 Angle PID Gain--------------------------------------------------------------------------------------------------------------*/
void Load_Angle_PID_Gain(void)
{
	HAL_FLASH_Unlock();
	
	EE_ReadVariable(PID_Angle_Pitch_P_EE_Address,&PID_Pitch_Angle.Buff.p);
	EE_ReadVariable(PID_Angle_Pitch_I_EE_Address,&PID_Pitch_Angle.Buff.i);	
	EE_ReadVariable(PID_Angle_Pitch_D_EE_Address,&PID_Pitch_Angle.Buff.d);
	
	EE_ReadVariable(PID_Angle_Roll_P_EE_Address,&PID_Roll_Angle.Buff.p);
	EE_ReadVariable(PID_Angle_Roll_I_EE_Address,&PID_Roll_Angle.Buff.i);	
	EE_ReadVariable(PID_Angle_Roll_D_EE_Address,&PID_Roll_Angle.Buff.d);
	
	EE_ReadVariable(PID_Angle_Yaw_P_EE_Address,&PID_Yaw_Angle.Buff.p);
	EE_ReadVariable(PID_Angle_Yaw_I_EE_Address,&PID_Yaw_Angle.Buff.i);	
	EE_ReadVariable(PID_Angle_Yaw_D_EE_Address,&PID_Yaw_Angle.Buff.d);
	
	PID_Pitch_Angle.gain.p = PID_Pitch_Angle.Buff.p*0.001;
	PID_Pitch_Angle.gain.i = PID_Pitch_Angle.Buff.i*0.001;
	PID_Pitch_Angle.gain.d = PID_Pitch_Angle.Buff.d*0.001;
	
	PID_Roll_Angle.gain.p = PID_Roll_Angle.Buff.p*0.001;
	PID_Roll_Angle.gain.i = PID_Roll_Angle.Buff.i*0.001;
	PID_Roll_Angle.gain.d = PID_Roll_Angle.Buff.d*0.001;
	
	PID_Yaw_Angle.gain.p = PID_Yaw_Angle.Buff.p*0.001;
	PID_Yaw_Angle.gain.i = PID_Yaw_Angle.Buff.i*0.001;
	PID_Yaw_Angle.gain.d = PID_Yaw_Angle.Buff.d*0.001;
	
	HAL_FLASH_Lock();
}
/*---------------------------------------------------------------------------------------------------------------------------------*/
/*載入 Angular Velocity PID Gain---------------------------------------------------------------------------------------------------*/
void Load_Angular_Velocity_PID_Gain(void)
{
	HAL_FLASH_Unlock();
	EE_ReadVariable(PID_Angular_Velocity_Pitch_P_EE_Address,&PID_Pitch.Buff.p);
	EE_ReadVariable(PID_Angular_Velocity_Pitch_I_EE_Address,&PID_Pitch.Buff.i);	
	EE_ReadVariable(PID_Angular_Velocity_Pitch_D_EE_Address,&PID_Pitch.Buff.d);
	
	EE_ReadVariable(PID_Angular_Velocity_Roll_P_EE_Address,&PID_Roll.Buff.p);
	EE_ReadVariable(PID_Angular_Velocity_Roll_I_EE_Address,&PID_Roll.Buff.i);	
	EE_ReadVariable(PID_Angular_Velocity_Roll_D_EE_Address,&PID_Roll.Buff.d);
	
	EE_ReadVariable(PID_Angular_Velocity_Yaw_P_EE_Address,&PID_Yaw.Buff.p);
	EE_ReadVariable(PID_Angular_Velocity_Yaw_I_EE_Address,&PID_Yaw.Buff.i);	
	EE_ReadVariable(PID_Angular_Velocity_Yaw_D_EE_Address,&PID_Yaw.Buff.d);
	
	PID_Pitch.gain.p = PID_Pitch.Buff.p*0.001;
	PID_Pitch.gain.i = PID_Pitch.Buff.i*0.001;
	PID_Pitch.gain.d = PID_Pitch.Buff.d*0.001;
	
	PID_Roll.gain.p = PID_Roll.Buff.p*0.001;
	PID_Roll.gain.i = PID_Roll.Buff.i*0.001;
	PID_Roll.gain.d = PID_Roll.Buff.d*0.001;
	
	PID_Yaw.gain.p = PID_Yaw.Buff.p*0.001;
	PID_Yaw.gain.i = PID_Yaw.Buff.i*0.001;
	PID_Yaw.gain.d = PID_Yaw.Buff.d*0.001;
	
	HAL_FLASH_Lock();
}
/* 角度PID控制 Pitch Roll----------------------------------------------------------------------------------------------------------*/
void PID_Angle_controller(PID_TypeDef* tmpPID)
	{
		tmpPID->error.p = tmpPID->Cmd - tmpPID->FB;
		tmpPID->error.i += tmpPID->error.p;
		tmpPID->error.d -= tmpPID->lest_error.p;

		/*積分鎖飽和*/ 
		if(tmpPID->error.i > 50)tmpPID->error.i = 50; 
		if(tmpPID->error.i < -50)tmpPID->error.i = -50;

		/*PID output*/
		tmpPID->output = tmpPID->error.p * tmpPID->gain.p + tmpPID->error.i * tmpPID->gain.i - tmpPID->error.d * tmpPID->gain.d;

		/*PWM鎖飽和*/
		if(tmpPID->output > 400)tmpPID->output=400;
		else if(tmpPID->output < -400)tmpPID->output= -400;

		tmpPID->lest_error.p = tmpPID->error.p;	
	}

/* 角速度PID控制 Pitch Roll--------------------------------------------------------------------------------------------------------*/	
void PID_controller(PID_TypeDef* tmpPID)
	{
		tmpPID->error.p = tmpPID->Cmd - tmpPID->FB;
		tmpPID->error.i += tmpPID->error.p;
		tmpPID->error.d -= tmpPID->lest_error.p;

		/*積分鎖飽和*/ 
		if(tmpPID->error.i > 60)tmpPID->error.i = 60; 
		if(tmpPID->error.i < -60)tmpPID->error.i = -60;
			 
		/*PID output*/	
		tmpPID->output = tmpPID->error.p * tmpPID->gain.p + tmpPID->error.i * tmpPID->gain.i + tmpPID->error.d * tmpPID->gain.d;;
		   
		/*鎖飽和*/
		if(tmpPID->output > 400)tmpPID->output=400;
		else if(tmpPID->output < -400)tmpPID->output= -400;

		tmpPID->lest_error.p = tmpPID->error.p;	
		
}

/* 高度控制PID--------------------------------------------------------------------------------------------------------*/	
void PID_Throttle_controller(PID_TypeDef* tmpPID)
	{
		tmpPID->error.p =  tmpPID->FB - tmpPID->Cmd;
		tmpPID->error.d -= tmpPID->lest_error.p;

		/*PID output*/	
		tmpPID->output = tmpPID->error.p * tmpPID->gain.p + tmpPID->error.d * tmpPID->gain.d;;
		   
		/*鎖飽和*/
		if(tmpPID->output > 200)tmpPID->output=200;
		else if(tmpPID->output < -200)tmpPID->output= -200;

		tmpPID->lest_error.p = tmpPID->error.p;	
}
	
	
	

