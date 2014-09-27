#ifndef __OLED_H__
#define __OLED_H__

#include "sys.h"
#include "gpio.h"
//#include "delay.h"

#endif /* __OLED_H__ */

void oled_init();
void Display_Info();
void Clean_Screen();

void fill_picture(unsigned char fill_Data);
void display_graphic_5x7(uint32_t page,uint16_t column,uint8_t *dp);
void Picture(void);
void Picture_2(void);
//void IIC_Start(void);
//void IIC_Stop(void);
void Write_IIC_Command(unsigned char IIC_Command);
void Write_IIC_Data(unsigned char IIC_Data);
void Write_IIC_Byte(unsigned char IIC_Byte);
void lcd_address(unsigned char page,unsigned char column);
void Set_Pos(uint8_t x,uint8_t y);
void Display_Pixel(uint8_t y,uint8_t *pixel,uint8_t *pixel2);
void Display_Char(uint8_t x,uint8_t y,uint8_t dat);
void Display_String(uint8_t x,uint8_t y,uint8_t *dat);
void Display_Number(uint8_t x,uint8_t y,int num);
void ValueOf(int m,uint8_t *str);
