#include "Rotary_Sw.h"

GPIO_PinState RS_ReadKey(GPIO_TypeDef * i_PORT, uint32_t i_PIN)
{
	GPIO_PinState iVal = GPIO_PIN_RESET;
	iVal = HAL_GPIO_ReadPin(i_PORT, i_PIN);
	HAL_Delay(10);
	iVal &= HAL_GPIO_ReadPin(i_PORT, i_PIN);
	return iVal;
	
}
