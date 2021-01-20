#include "adc.h"
#include "Battery.h"
#include "stm32f4xx_hal.h"

ADC_battery Battery;

/*ADC¹qÀ£°»´ú-----------------------------------------------------------------*/
float ADC_Vaule(void)
{
    HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
	
		Battery.Value = HAL_ADC_GetValue(&hadc1);
	
		Battery.Voltage = ((Battery.Value/4095.0)*20.0f);
//		if(Battery.Voltage>12)Battery.Voltage=Battery.Voltage;
//		else Battery.Voltage = Battery.Voltage - 0.2;
    return Battery.Voltage;
}
/*----------------------------------------------------------------------------*/