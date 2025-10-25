#include "../Inc/main.h"
#include "../Inc/timer.h"

volatile uint32_t stopwatch_time = 0;
volatile uint8_t stopwatch_running = 0;
volatile uint32_t system_ticks = 0;

void Timer1_Init(void){
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
