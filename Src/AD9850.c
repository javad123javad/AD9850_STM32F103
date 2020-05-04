#include "AD9850.h"

static AD_CFG g_AD_CFG;

static void AD_EN_Serial(void)
{
	/* Enable serial load sequence */
	HAL_GPIO_WritePin(g_AD_CFG.W_CLK_PORT, g_AD_CFG.W_CLK_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(g_AD_CFG.FU_FD_PORT, g_AD_CFG.FU_FD_PIN, GPIO_PIN_RESET);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(g_AD_CFG.FU_FD_PORT, g_AD_CFG.FU_FD_PIN, GPIO_PIN_SET);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(g_AD_CFG.FU_FD_PORT, g_AD_CFG.FU_FD_PIN, GPIO_PIN_RESET);
	//HAL_Delay(1);
}
void AD_Init(AD_CFG * const i_ad_cfg){
	
	g_AD_CFG.W_CLK_PORT = i_ad_cfg->W_CLK_PORT;
	g_AD_CFG.FU_FD_PORT = i_ad_cfg->FU_FD_PORT;
	g_AD_CFG.DATA_PORT  = i_ad_cfg->DATA_PORT;
	
	g_AD_CFG.W_CLK_PIN = i_ad_cfg->W_CLK_PIN;
	g_AD_CFG.FU_FD_PIN = i_ad_cfg->FU_FD_PIN;
	g_AD_CFG.DATA_PIN  = i_ad_cfg->DATA_PIN;
	
}

void AD_Send_Cmd(const AD_MSG * const i_msg)
{
	AD_EN_Serial();
	for (int i = 0; i<32; i++)
  {
    HAL_GPIO_WritePin(g_AD_CFG.DATA_PORT, g_AD_CFG.DATA_PIN, (GPIO_PinState)((i_msg->freq >> i) & 0x01));
		HAL_GPIO_WritePin(g_AD_CFG.W_CLK_PORT, g_AD_CFG.W_CLK_PIN, GPIO_PIN_SET);
		//HAL_Delay(1);
		HAL_GPIO_WritePin(g_AD_CFG.W_CLK_PORT, g_AD_CFG.W_CLK_PIN, GPIO_PIN_RESET);
		//HAL_Delay(1);
   }
	for (int i = 0; i<8; i++)
	{
    HAL_GPIO_WritePin(g_AD_CFG.DATA_PORT, g_AD_CFG.DATA_PIN, (GPIO_PinState)((i_msg->ctrl_phase >> i) & 0x01));
		HAL_GPIO_WritePin(g_AD_CFG.W_CLK_PORT, g_AD_CFG.W_CLK_PIN, GPIO_PIN_SET);
		//HAL_Delay(1);
		HAL_GPIO_WritePin(g_AD_CFG.W_CLK_PORT, g_AD_CFG.W_CLK_PIN, GPIO_PIN_RESET);
		//HAL_Delay(1);
   }
		HAL_GPIO_WritePin(g_AD_CFG.FU_FD_PORT, g_AD_CFG.FU_FD_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(g_AD_CFG.W_CLK_PORT, g_AD_CFG.W_CLK_PIN, GPIO_PIN_RESET);
		//HAL_Delay(1);
		HAL_GPIO_WritePin(g_AD_CFG.FU_FD_PORT, g_AD_CFG.FU_FD_PIN, GPIO_PIN_RESET);
}
void AD_Power_Down(void)
{
	AD_MSG msg= {0,0x04};
	AD_Send_Cmd(&msg);
}
void AD_Set_Freq(uint32_t i_freq, uint8_t i_phase)
{
	AD_MSG msg={0,0};
	
	msg.freq = (i_freq * 4294967296)/OSC_FREQ;
	AD_Send_Cmd(&msg);
}
