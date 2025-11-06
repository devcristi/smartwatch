#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

extern volatile uint32_t stopwatch_time;
extern volatile uint8_t stopwatch_running;
extern volatile uint32_t system_ticks;

void Timer1_Init(void);

#endif // TIMER_H
