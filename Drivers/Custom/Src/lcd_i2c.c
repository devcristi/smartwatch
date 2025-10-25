#include "../../Inc/main.h"
#include "../../Inc/i2c.h"
#include "../Inc/lcd_i2c.h"

static void LCD_Write4(uint8_t nibble, uint8_t flags){
	uint8_t d = (nibble<<4) | flags | LCD_BL;
	TWI_Start();
	TWI_Write((LCD_ADDR<<1)|0);
	TWI_Write(d | LCD_EN);
	_delay_us(1);
	TWI_Write(d & ~LCD_EN);
	TWI_Stop();
	_delay_us(50);
}

void LCD_Command(uint8_t cmd){
	LCD_Write4(cmd>>4, 0);
	LCD_Write4(cmd&0x0F, 0);
}

void LCD_Data(uint8_t data){
	LCD_Write4(data>>4, LCD_RS);
	LCD_Write4(data&0x0F, LCD_RS);
}

void LCD_Init(void){
	_delay_ms(50);
	LCD_Write4(0x03, 0); _delay_ms(5);
	LCD_Write4(0x03, 0); _delay_ms(5);
	LCD_Write4(0x03, 0); _delay_ms(1);
	LCD_Write4(0x02, 0);
	LCD_Command(0x28);
	LCD_Command(0x0C);
	LCD_Command(0x06);
	LCD_Command(0x01);
	_delay_ms(2);
}

void LCD_SetCursor(uint8_t col, uint8_t row){
	LCD_Command(0x80 | (row ? 0x40 : 0x00) | col);
}

void LCD_Clear(void){
	LCD_Command(0x01);
	_delay_ms(2);
}

void LCD_PrintNum(uint8_t val){
	if(val >= 10) LCD_Data('0' + val/10);
	else LCD_Data('0');
	LCD_Data('0' + val%10);
}

void LCD_PrintStr(const char* str){
	while(*str) LCD_Data(*str++);
}

void LCD_PrintFloat(float val){
	int8_t integer = (int8_t)val;
	int8_t fraction = (int8_t)((val - integer) * 100);
	if(fraction < 0) fraction = -fraction;
	
	if(integer < 0){
		LCD_Data('-');
		integer = -integer;
	}
	LCD_PrintNum(integer);
	LCD_Data('.');
	if(fraction < 10) LCD_Data('0');
	LCD_PrintNum(fraction);
}
