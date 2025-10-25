#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

/* ================= I2C ================= */
#define DS3231_ADDR 0x68
#define LCD_ADDR    0x27

#define LCD_BL 0x08
#define LCD_EN 0x04
#define LCD_RW 0x02
#define LCD_RS 0x01

static inline void TWI_Init(void){
	TWSR = 0x00;
	TWBR = 72;
	TWCR = (1<<TWEN);
}

static inline void TWI_Start(void){
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
}

static inline void TWI_Stop(void){
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	_delay_us(10);
}

static inline void TWI_Write(uint8_t d){
	TWDR = d;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
}

static inline uint8_t TWI_ReadACK(void){
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}

static inline uint8_t TWI_ReadNACK(void){
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}

/* ================= LCD ================= */
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

static void LCD_Command(uint8_t cmd){
	LCD_Write4(cmd>>4,0);
	LCD_Write4(cmd&0x0F,0);
}

static void LCD_Data(uint8_t data){
	LCD_Write4(data>>4,LCD_RS);
	LCD_Write4(data&0x0F,LCD_RS);
}

static void LCD_Init(void){
	_delay_ms(50);
	LCD_Write4(0x03,0); _delay_ms(5);
	LCD_Write4(0x03,0); _delay_ms(5);
	LCD_Write4(0x03,0); _delay_ms(1);
	LCD_Write4(0x02,0);
	LCD_Command(0x28);
	LCD_Command(0x0C);
	LCD_Command(0x06);
	LCD_Command(0x01);
	_delay_ms(2);
}

static void LCD_SetCursor(uint8_t col,uint8_t row){
	LCD_Command(0x80 | (row?0x40:0x00) | col);
}

static void LCD_Clear(void){
	LCD_Command(0x01);
	_delay_ms(2);
}

static void LCD_PrintNum(uint8_t val){
	if(val>=10) LCD_Data('0'+val/10);
	else LCD_Data('0');
	LCD_Data('0'+val%10);
}

static void LCD_PrintStr(const char* s){
	while(*s) LCD_Data(*s++);
}

static void LCD_PrintFloat(float val){
	int8_t i = (int8_t)val;
	int8_t f = (int8_t)((val - i)*100);
	if(f < 0) f = -f;
	
	if(i < 0){
		LCD_Data('-');
		i = -i;
	}
	LCD_PrintNum(i);
	LCD_Data('.');
	if(f<10) LCD_Data('0');
	LCD_PrintNum(f);
}

/* ================= DS3231 RTC ================= */
static uint8_t BCD2DEC(uint8_t b){
	return ((b>>4)*10)+(b&0x0F);
}

static void DS3231_GetTime(uint8_t *h,uint8_t *m,uint8_t *s){
	TWI_Start();
	TWI_Write((DS3231_ADDR<<1)|0);
	TWI_Write(0x00);
	TWI_Start();
	TWI_Write((DS3231_ADDR<<1)|1);
	*s = BCD2DEC(TWI_ReadACK() & 0x7F);
	*m = BCD2DEC(TWI_ReadACK() & 0x7F);
	*h = BCD2DEC(TWI_ReadNACK() & 0x3F);
	TWI_Stop();
}

static float DS3231_GetTemp(void){
	TWI_Start();
	TWI_Write((DS3231_ADDR<<1)|0);
	TWI_Write(0x11);
	TWI_Start();
	TWI_Write((DS3231_ADDR<<1)|1);
	int8_t msb = TWI_ReadACK();
	uint8_t lsb = TWI_ReadNACK();
	TWI_Stop();
	return msb + ((lsb>>6)*0.25);
}

/* ================= TIMER1 pentru Cronometru ================= */
volatile uint32_t stopwatch_time = 0;
volatile uint8_t stopwatch_running = 0;
volatile uint32_t system_ticks = 0;

static void Timer1_Init(void){
	TCCR1A = 0x00;
	TCCR1B = (1<<WGM12) | (1<<CS12);
	OCR1A = 624;
	TIMSK1 = (1<<OCIE1A);
}

ISR(TIMER1_COMPA_vect){
	system_ticks++;
	if(stopwatch_running){
		stopwatch_time++;
	}
}

/* ================= 3 BUTOANE - INT4, INT5, INT2 ================= */
#define BTN_NEXT_PIN     PE4  // INT4 - Pin 2 (PWM) - Meniu ÎNAINTE
#define BTN_PREV_PIN     PE5  // INT5 - Pin 3 (PWM) - Meniu ÎNAPOI
#define BTN_ACTION_PIN   PD2  // INT2 - Pin 19 (RXD1) - START/PAUSE/RESET cronometru

#define TOTAL_MENUS 3

volatile uint8_t btn_next_event = 0;
volatile uint8_t btn_prev_event = 0;
volatile uint8_t btn_action_event = 0;

// INT4 - Buton NEXT
ISR(INT4_vect){
	btn_next_event = 1;
}

// INT5 - Buton PREV
ISR(INT5_vect){
	btn_prev_event = 1;
}

// INT2 - Buton ACTION
ISR(INT2_vect){
	btn_action_event = 1;
}

static void Buttons_Init(void){
	// PE4, PE5 ca input cu pull-up
	DDRE &= ~((1<<BTN_NEXT_PIN) | (1<<BTN_PREV_PIN));
	PORTE |= (1<<BTN_NEXT_PIN) | (1<<BTN_PREV_PIN);
	
	// PD2 ca input cu pull-up
	DDRD &= ~(1<<BTN_ACTION_PIN);
	PORTD |= (1<<BTN_ACTION_PIN);
	
	// INT4, INT5 pe falling edge
	EICRB |= (1<<ISC41) | (1<<ISC51);
	EICRB &= ~((1<<ISC40) | (1<<ISC50));
	
	// INT2 pe falling edge
	EICRA |= (1<<ISC21);
	EICRA &= ~(1<<ISC20);
	
	// Clear întreruperi pendinte
	EIFR |= (1<<INTF4) | (1<<INTF5) | (1<<INTF2);
	
	// Activează INT4, INT5, INT2
	EIMSK |= (1<<INT4) | (1<<INT5) | (1<<INT2);
	
	sei();
}

/* ================= Cronometru ================= */
static uint8_t menu_index = 0;
static uint8_t stopwatch_state = 0; // 0=stopped, 1=running, 2=paused

static void Stopwatch_Display(void){
	uint32_t time_units = stopwatch_time;
	uint16_t minutes = (time_units / 6000);
	uint8_t seconds = ((time_units % 6000) / 100);
	uint8_t centisec = (time_units % 100);
	
	LCD_SetCursor(0,0);
	if(stopwatch_state == 0){
		LCD_PrintStr("  STOPWATCH     ");
		} else if(stopwatch_state == 1){
		LCD_PrintStr("  RUNNING...    ");
		} else {
		LCD_PrintStr("  PAUSED        ");
	}
	
	LCD_SetCursor(0,1);
	LCD_PrintStr("  ");
	if(minutes < 10) LCD_Data('0');
	LCD_PrintNum(minutes & 0xFF);
	LCD_Data(':');
	LCD_PrintNum(seconds);
	LCD_Data(':');
	LCD_PrintNum(centisec);
	LCD_PrintStr("  ");
}

/* ================= Button Handlers ================= */
static void Handle_Next_Button(void){
	static uint32_t last_press = 0;
	uint32_t now = system_ticks;
	
	// Debounce 50ms
	if((now - last_press) < 5) return;
	last_press = now;
	
	// Navighează înainte (wrap around)
	menu_index = (menu_index + 1) % TOTAL_MENUS;
	LCD_Clear();
}

static void Handle_Prev_Button(void){
	static uint32_t last_press = 0;
	uint32_t now = system_ticks;
	
	// Debounce 50ms
	if((now - last_press) < 5) return;
	last_press = now;
	
	// Navighează înapoi (wrap around)
	if(menu_index == 0){
		menu_index = TOTAL_MENUS - 1;
		} else {
		menu_index--;
	}
	LCD_Clear();
}

/* ================= MAIN ================= */
int main(void){
	TWI_Init();
	Timer1_Init();
	LCD_Init();
	Buttons_Init();

	uint8_t h, m, s;
	float temp;
	uint8_t last_menu = 0xFF;

	// Variabile pentru butonul ACTION
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
			}
		}
		
		// ========== BUTON PREV (înapoi) ==========
		if(btn_prev_event){
			btn_prev_event = 0;
			_delay_ms(50);
			if(!(PINE & (1<<BTN_PREV_PIN))){
				Handle_Prev_Button();
			}
		}
		
		// ========== BUTON ACTION - Acum pe PD2 (fără conflict I2C!) ==========
		if(menu_index == 2){  // Doar în modul STOPWATCH
			
			// Detectează întreruperea (falling edge)
			if(btn_action_event){
				btn_action_event = 0;
				_delay_ms(50); // Debounce
				
				// Verifică dacă butonul e încă apăsat
				if(!(PIND & (1<<BTN_ACTION_PIN))){
					btn_was_pressed = 1;
					btn_press_start_time = system_ticks;
					reset_done = 0;
				}
			}
			
			// Monitorizează apăsarea lungă
			if(btn_was_pressed){
				// Verifică starea butonului direct din registru
				uint8_t btn_still_pressed = !(PIND & (1<<BTN_ACTION_PIN));
				
				if(btn_still_pressed){
					uint32_t hold_duration = system_ticks - btn_press_start_time;
					
					// După 1 secundă -> RESET
					if(hold_duration >= 100 && !reset_done){
						stopwatch_time = 0;
						stopwatch_running = 0;
						stopwatch_state = 0;
						reset_done = 1;
					}
				}
				else {
					// Butonul a fost eliberat
					uint32_t press_duration = system_ticks - btn_press_start_time;
					
					// Apăsare scurtă < 1s -> START/PAUSE
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
			// Resetează starea când nu suntem în cronometru
			btn_was_pressed = 0;
			btn_action_event = 0;
		}
		
		// ========== BUTON PREV ținut apăsat în modul TIME = RESET ceas la 00:00:00 ==========
		static uint32_t prev_hold_start = 0;
		static uint8_t prev_reset_triggered = 0;
		
		if(menu_index == 0 && !(PINE & (1<<BTN_PREV_PIN))){
			// Butonul PREV e apăsat în modul TIME
			if(prev_hold_start == 0){
				prev_hold_start = system_ticks;
				prev_reset_triggered = 0;
			}
			
			uint32_t hold_time = system_ticks - prev_hold_start;
			
			// Dacă e ținut >1s, RESET ceas la 00:00:00
			if(hold_time > 100 && !prev_reset_triggered){
				prev_reset_triggered = 1;
				// RESET RTC la 00:00:00
				TWI_Start();
				TWI_Write((DS3231_ADDR<<1)|0);
				TWI_Write(0x00); // Adresa secundelor
				TWI_Write(0x00); // Secunde = 0
				TWI_Write(0x00); // Minute = 0
				TWI_Write(0x00); // Ore = 0
				TWI_Stop();
			}
			} else {
			prev_hold_start = 0;
		}

		// ========== UPDATE DISPLAY (doar dacă butonul ACTION nu e apăsat!) ==========
		// IMPORTANT: Nu face I2C dacă butonul SCL e apăsat!
		if(!btn_was_pressed){
			if(last_menu != menu_index){
				last_menu = menu_index;
				LCD_Clear();
			}

			if(menu_index == 0){
				// TIME mode
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
				// TEMPERATURE mode
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
				// STOPWATCH mode
				Stopwatch_Display();
			}
		}

		_delay_ms(100);
	}

	return 0;
}