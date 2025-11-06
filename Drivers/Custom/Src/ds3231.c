#include "../../Inc/main.h"
#include "../../Inc/i2c.h"
#include "../Inc/ds3231.h"

static uint8_t BCD2DEC(uint8_t bcd){
	return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

void DS3231_GetTime(uint8_t *hours, uint8_t *minutes, uint8_t *seconds){
	TWI_Start();
	TWI_Write((DS3231_ADDR << 1) | 0);
	TWI_Write(0x00);
	TWI_Start();
	TWI_Write((DS3231_ADDR << 1) | 1);
	*seconds = BCD2DEC(TWI_ReadACK() & 0x7F);
	*minutes = BCD2DEC(TWI_ReadACK() & 0x7F);
	*hours = BCD2DEC(TWI_ReadNACK() & 0x3F);
	TWI_Stop();
}

float DS3231_GetTemp(void){
	TWI_Start();
	TWI_Write((DS3231_ADDR << 1) | 0);
	TWI_Write(0x11);
	TWI_Start();
	TWI_Write((DS3231_ADDR << 1) | 1);
	int8_t msb = TWI_ReadACK();
	uint8_t lsb = TWI_ReadNACK();
	TWI_Stop();
	return msb + ((lsb >> 6) * 0.25);
}

void DS3231_ResetTime(void){
	TWI_Start();
	TWI_Write((DS3231_ADDR << 1) | 0);
	TWI_Write(0x00);
	TWI_Write(0x00);
	TWI_Write(0x00);
	TWI_Write(0x00);
	TWI_Stop();
}
