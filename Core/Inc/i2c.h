#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void TWI_Init(void);
void TWI_Start(void);
void TWI_Stop(void);
void TWI_Write(uint8_t data);
uint8_t TWI_ReadACK(void);
uint8_t TWI_ReadNACK(void);

#endif // I2C_H
