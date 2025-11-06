#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdint.h>

#define BTN_NEXT_PIN     PE4
#define BTN_PREV_PIN     PE5
#define BTN_ACTION_PIN   PD2

#define TOTAL_MENUS 3

extern volatile uint8_t btn_next_event;
extern volatile uint8_t btn_prev_event;
extern volatile uint8_t btn_action_event;

void Buttons_Init(void);
void Handle_Next_Button(void);
void Handle_Prev_Button(void);

#endif // BUTTONS_H
