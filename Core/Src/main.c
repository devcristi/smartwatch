#include "../Inc/main.h"#include "main.h"

#include "../Inc/i2c.h"

#include "../Inc/buttons.h"int main(void) {

#include "../Inc/timer.h"    // Initialize system

#include "../Inc/stopwatch.h"    while (1) {

#include "../../Drivers/Custom/Inc/lcd_i2c.h"        // Main loop

#include "../../Drivers/Custom/Inc/ds3231.h"    }

    return 0;

uint8_t menu_index = 0;}

int main(void){
	TWI_Init();
	Timer1_Init();
	LCD_Init();
	Buttons_Init();

	uint8_t h, m, s;
	float temp;
	uint8_t last_menu = 0xFF;

	static uint8_t btn_was_pressed = 0;
	static uint32_t btn_press_start_time = 0;
	static uint8_t reset_done = 0;

	LCD_Clear();

	while(1){
		// ========== BUTON NEXT (înainte) ==========
		if(btn_next_event){
			btn_next_event = 0;
			_delay_ms(50);
			if(!(PINE & (1<<BTN_NEXT_PIN))){
				Handle_Next_Button();
				LCD_Clear();
			}
		}
		
		// ========== BUTON PREV (înapoi) ==========
		if(btn_prev_event){
			btn_prev_event = 0;
			_delay_ms(50);
			if(!(PINE & (1<<BTN_PREV_PIN))){
				Handle_Prev_Button();
				LCD_Clear();
			}
		}
		
		// ========== BUTON ACTION - START/PAUSE/RESET cronometru ==========
		if(menu_index == 2){
			if(btn_action_event){
				btn_action_event = 0;
				_delay_ms(50);
				
				if(!(PIND & (1<<BTN_ACTION_PIN))){
					btn_was_pressed = 1;
					btn_press_start_time = system_ticks;
					reset_done = 0;
				}
			}
			
			if(btn_was_pressed){
				uint8_t btn_still_pressed = !(PIND & (1<<BTN_ACTION_PIN));
				
				if(btn_still_pressed){
					uint32_t hold_duration = system_ticks - btn_press_start_time;
					
					if(hold_duration >= 100 && !reset_done){
						stopwatch_time = 0;
						stopwatch_running = 0;
						stopwatch_state = 0;
						reset_done = 1;
					}
				}
				else {
					uint32_t press_duration = system_ticks - btn_press_start_time;
					
					if(press_duration < 100){
						if(stopwatch_state == 0 || stopwatch_state == 2){
							stopwatch_running = 1;
							stopwatch_state = 1;
						} else {
							stopwatch_running = 0;
							stopwatch_state = 2;
						}
					}
					
					btn_was_pressed = 0;
				}
			}
		}
		else {
			btn_was_pressed = 0;
			btn_action_event = 0;
		}
		
		// ========== BUTON PREV ținut apăsat în modul TIME = RESET ceas ==========
		static uint32_t prev_hold_start = 0;
		static uint8_t prev_reset_triggered = 0;
		
		if(menu_index == 0 && !(PINE & (1<<BTN_PREV_PIN))){
			if(prev_hold_start == 0){
				prev_hold_start = system_ticks;
				prev_reset_triggered = 0;
			}
			
			uint32_t hold_time = system_ticks - prev_hold_start;
			
			if(hold_time > 100 && !prev_reset_triggered){
				prev_reset_triggered = 1;
				DS3231_ResetTime();
			}
		} else {
			prev_hold_start = 0;
		}

		// ========== UPDATE DISPLAY ==========
		if(!btn_was_pressed){
			if(last_menu != menu_index){
				last_menu = menu_index;
				LCD_Clear();
			}

			if(menu_index == 0){
				LCD_SetCursor(0,0);
				LCD_PrintStr("    TIME        ");
				
				DS3231_GetTime(&h, &m, &s);
				LCD_SetCursor(0,1);
				LCD_PrintStr("   ");
				LCD_PrintNum(h);
				LCD_Data(':');
				LCD_PrintNum(m);
				LCD_Data(':');
				LCD_PrintNum(s);
				LCD_PrintStr("   ");
				
			} else if(menu_index == 1){
				LCD_SetCursor(0,0);
				LCD_PrintStr("  TEMPERATURE   ");
				
				temp = DS3231_GetTemp();
				LCD_SetCursor(0,1);
				LCD_PrintStr("   ");
				LCD_PrintFloat(temp);
				LCD_Data(0xDF);
				LCD_Data('C');
				LCD_PrintStr("   ");
				
			} else {
				Stopwatch_Display();
			}
		}

		_delay_ms(100);
	}

	return 0;
}
