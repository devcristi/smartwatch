#include "../Inc/main.h"
#include "../Inc/buttons.h"
#include "../Inc/timer.h"

volatile uint8_t btn_next_event = 0;
volatile uint8_t btn_prev_event = 0;
volatile uint8_t btn_action_event = 0;

extern uint8_t menu_index;
extern volatile uint32_t system_ticks;

ISR(INT4_vect){
	btn_next_event = 1;
}

ISR(INT5_vect){
	btn_prev_event = 1;
}

ISR(INT2_vect){
	btn_action_event = 1;
}

void Buttons_Init(void){
	DDRE &= ~((1<<BTN_NEXT_PIN) | (1<<BTN_PREV_PIN));
	PORTE |= (1<<BTN_NEXT_PIN) | (1<<BTN_PREV_PIN);
	
	DDRD &= ~(1<<BTN_ACTION_PIN);
	PORTD |= (1<<BTN_ACTION_PIN);
	
	EICRB |= (1<<ISC41) | (1<<ISC51);
	EICRB &= ~((1<<ISC40) | (1<<ISC50));
	
	EICRA |= (1<<ISC21);
	EICRA &= ~(1<<ISC20);
	
	EIFR |= (1<<INTF4) | (1<<INTF5) | (1<<INTF2);
	EIMSK |= (1<<INT4) | (1<<INT5) | (1<<INT2);
	
	sei();
}

void Handle_Next_Button(void){
	static uint32_t last_press = 0;
	uint32_t now = system_ticks;
	
	if((now - last_press) < 5) return;
	last_press = now;
	
	menu_index = (menu_index + 1) % TOTAL_MENUS;
}

void Handle_Prev_Button(void){
	static uint32_t last_press = 0;
	uint32_t now = system_ticks;
	
	if((now - last_press) < 5) return;
	last_press = now;
	
	if(menu_index == 0){
		menu_index = TOTAL_MENUS - 1;
	} else {
		menu_index--;
	}
}
