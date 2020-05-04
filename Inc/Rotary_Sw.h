#ifndef _ROTART_SW_H_
#define _ROTART_SW_H_
#include "stm32f1xx_hal.h"

GPIO_PinState RS_ReadKey(GPIO_TypeDef * i_PORT, uint32_t i_PIN);

#endif
