#include "../Inc/main.h"
#include "../Inc/i2c.h"

void TWI_Init(void){
	TWSR = 0x00;
	TWBR = 72;
	TWCR = (1<<TWEN);
}

void TWI_Start(void){
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
}

void TWI_Stop(void){
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
	_delay_us(10);
}

void TWI_Write(uint8_t d){
	TWDR = d;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
}

uint8_t TWI_ReadACK(void){
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}

uint8_t TWI_ReadNACK(void){
	TWCR = (1<<TWINT)|(1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;
}
