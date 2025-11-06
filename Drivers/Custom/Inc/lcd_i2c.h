#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>

#define LCD_ADDR    0x27
#define LCD_BL 0x08
#define LCD_EN 0x04
#define LCD_RW 0x02
#define LCD_RS 0x01

void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t col, uint8_t row);
void LCD_PrintStr(const char* str);
void LCD_PrintNum(uint8_t val);
void LCD_PrintFloat(float val);
void LCD_Command(uint8_t cmd);
void LCD_Data(uint8_t data);

#endif // LCD_I2C_H
