#ifndef _I2C_LCD_H_
#define _I2C_LCD_H_
#include "stm32f1xx_hal.h"

typedef I2C_HandleTypeDef LCD_I2C_PORT;
// change this according to ur setup
#define SLAVE_ADDRESS_LCD 0x4E 

void lcd_cfg( LCD_I2C_PORT  i_lcd_i2c);
void lcd_send_cmd(char cmd);
void lcd_send_data (char data);
void lcd_init (void);
void lcd_send_string (char *str);



#endif
