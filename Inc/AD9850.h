#ifndef _AD9850_H_
#define _AD9850_H_
#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct _AD_DATA_PACK
{
		uint32_t	freq;
		uint8_t		ctrl_phase;
}AD_MSG;

typedef struct _AD_PIN_CFG
{
	/* Port definition */
	GPIO_TypeDef * W_CLK_PORT;
	GPIO_TypeDef * FU_FD_PORT;
	GPIO_TypeDef * DATA_PORT;
	/* Pin definition */
	uint32_t 			 W_CLK_PIN;
	uint32_t			 FU_FD_PIN;
	uint32_t			 DATA_PIN;
	
}AD_CFG;
	
#define OSC_FREQ	125000000U /* AD9850 clock frequency in Hz */


void AD_Init(AD_CFG * const i_ad_cfg); /* Powering up the module and putting it in serial mode protocol */
void AD_Send_Cmd(const AD_MSG * const i_msg);
void AD_Power_Down(void);
void AD_Set_Freq(uint32_t i_freq, uint8_t i_phase);

#endif
