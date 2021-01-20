#include "stm32f4xx_hal.h"
/*EEPRON Address------------------------------------------*/
/*0x00~0x25 is pid eeprom address*/
#define PID_Angle_Pitch_P_EE_Address 						0x00
#define PID_Angle_Pitch_I_EE_Address 						0x01
#define PID_Angle_Pitch_D_EE_Address 						0x02

#define PID_Angle_Roll_P_EE_Address  						0x03
#define PID_Angle_Roll_I_EE_Address  						0x04
#define PID_Angle_Roll_D_EE_Address  						0x05

#define PID_Angle_Yaw_P_EE_Address   						0x06
#define PID_Angle_Yaw_I_EE_Address   						0x07
#define PID_Angle_Yaw_D_EE_Address   						0x08

#define PID_Angular_Velocity_Pitch_P_EE_Address 0x09
#define PID_Angular_Velocity_Pitch_I_EE_Address 0x10
#define PID_Angular_Velocity_Pitch_D_EE_Address 0x11

#define PID_Angular_Velocity_Roll_P_EE_Address 	0x12
#define PID_Angular_Velocity_Roll_I_EE_Address 	0x13
#define PID_Angular_Velocity_Roll_D_EE_Address 	0x14

#define PID_Angular_Velocity_Yaw_P_EE_Address		0x15
#define PID_Angular_Velocity_Yaw_I_EE_Address 	0x16
#define PID_Angular_Velocity_Yaw_D_EE_Address 	0x17


/*-------------------------------------------------------*/

typedef struct PID_config{
    struct PID_error
		{
        float p;
        float d;
        float i;
    }error;
    struct PID_gain
		{
        float p;
        float d;
        float i;
    }gain;
    struct lest_error
		{
        float p;
        float d;
        float i;
    }lest_error;
		struct Buff
		{
			uint16_t p;
			uint16_t i;
			uint16_t d;
		}Buff;
		struct Out
		{
			float p;
			float i;
			float d;
		}Out;		
    float Cmd;
    float FB;
    float output;
		float Last_data;
}PID_TypeDef ;

void Load_Angular_Velocity_PID_Gain(void);
void Load_Angle_PID_Gain(void);
void PID_controller(PID_TypeDef* tmpPID);
void PID_Angle_controller(PID_TypeDef* tmpPID);
void PID_Throttle_controller(PID_TypeDef* tmpPID);
void PID_YAW_controller(PID_TypeDef* tmpPID);