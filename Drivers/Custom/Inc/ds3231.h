#ifndef DS3231_H
#define DS3231_H

#include <stdint.h>

#define DS3231_ADDR 0x68

void DS3231_GetTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);
float DS3231_GetTemp(void);
void DS3231_ResetTime(void);

#endif // DS3231_H
