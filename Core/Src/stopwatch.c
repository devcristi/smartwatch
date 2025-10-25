#include "../Inc/main.h"
#include "../Inc/stopwatch.h"
#include "../Inc/timer.h"
#include "../../Drivers/Custom/Inc/lcd_i2c.h"

uint8_t stopwatch_state = 0;

void Stopwatch_Display(void){
	uint32_t time_units = stopwatch_time;
	uint16_t minutes = (time_units / 6000);
	uint8_t seconds = ((time_units % 6000) / 100);
	uint8_t centisec = (time_units % 100);
	
	LCD_SetCursor(0, 0);
	if(stopwatch_state == 0){
		LCD_PrintStr("  STOPWATCH     ");
	} else if(stopwatch_state == 1){
		LCD_PrintStr("  RUNNING...    ");
	} else {
		LCD_PrintStr("  PAUSED        ");
	}
	
	LCD_SetCursor(0, 1);
	LCD_PrintStr("  ");
	if(minutes < 10) LCD_Data('0');
	LCD_PrintNum(minutes & 0xFF);
	LCD_Data(':');
	LCD_PrintNum(seconds);
	LCD_Data(':');
	LCD_PrintNum(centisec);
	LCD_PrintStr("  ");
}
