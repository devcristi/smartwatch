/*
 * main.c - FIXED ALARM VERSION
 *
 * Integrated smartwatch with MPU6050, DS3231 RTC, LCD 16x2, BME280, and ALARM
 * - FIXED: Alarm now triggers correctly and rings continuously
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* ================= CONFIG ================= */
#define MPU_READ_TICKS 10
#define MPU_PRINT_TICKS 100
#define BME_READ_TICKS 50
#define EMA_ALPHA 0.18f

// SLEEP MODE CONFIG
#define DIM_TIMEOUT_TICKS 800      
#define SLEEP_TIMEOUT_TICKS 1000   
#define MOTION_THRESHOLD 300       

// RTC SYNC CONFIG
#define ENABLE_RTC_SYNC 0

// BME280 DEBUG MODE
#define BME280_DEBUG 0

// ALTITUDE CALCULATION CONFIG
#define SEA_LEVEL_PRESSURE 1013.25f

// BUZZER CONFIG
#define BUZZER_PIN PB5  // Pin 11 on Arduino Mega (OC1A - PWM)

/* ================= UART (Serial) ================= */
#define BAUD 115200
#define UBRR_VAL ((F_CPU/(16UL*BAUD))-1)

static void UART_Init(void){
	UBRR0H = (uint8_t)(UBRR_VAL>>8);
	UBRR0L = (uint8_t)UBRR_VAL;
	UCSR0B = (1<<TXEN0)|(1<<RXEN0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

static void UART_Transmit(uint8_t data){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

static void UART_Print(const char* str){
	while(*str){
		UART_Transmit(*str++);
	}
}

static void UART_PrintInt(int16_t val){
	char buffer[8];
	sprintf(buffer, "%d", val);
	UART_Print(buffer);
}

static void UART_PrintUInt(uint16_t val){
	char buffer[8];
	sprintf(buffer, "%u", val);
	UART_Print(buffer);
}

static void UART_PrintLong(int32_t val){
	char buffer[12];
	sprintf(buffer, "%ld", val);
	UART_Print(buffer);
}

static void UART_PrintFloat(float val){
	char buffer[16];
	dtostrf(val, 6, 2, buffer);
	UART_Print(buffer);
}

static void UART_Println(const char* str){
	UART_Print(str);
	UART_Transmit('\r');
	UART_Transmit('\n');
}

/* ================= BUZZER ================= */
static void Buzzer_Init(void){
	DDRB |= (1<<BUZZER_PIN);
	PORTB &= ~(1<<BUZZER_PIN);
}

// Simple tone without Timer1 conflict
static void Buzzer_Tone_Simple(uint16_t frequency, uint16_t duration_ms){
	uint32_t period = 1000000UL / frequency;
	uint32_t cycles = (uint32_t)duration_ms * 1000UL / period;
	
	for(uint32_t i = 0; i < cycles; i++){
		PORTB |= (1<<BUZZER_PIN);
		for(uint16_t d = 0; d < period/2; d++) _delay_us(1);
		PORTB &= ~(1<<BUZZER_PIN);
		for(uint16_t d = 0; d < period/2; d++) _delay_us(1);
	}
}

static void Buzzer_Off(void){
	PORTB &= ~(1<<BUZZER_PIN);
}

static void Buzzer_Beep(void){
	Buzzer_Tone_Simple(2000, 100);
	Buzzer_Off();
}

static void Buzzer_Alarm_Pattern(void){
	// Quick alarm beep pattern
	Buzzer_Tone_Simple(1000, 150);
	_delay_ms(50);
	Buzzer_Tone_Simple(1500, 150);
	_delay_ms(50);
}

/* ================= I2C ================= */
#define DS3231_ADDR  0x68
#define LCD_ADDR     0x27
#define MPU6050_ADDR 0x69
#define BME280_ADDR  0x76

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

static void LCD_Dim(void){
	TWI_Start();
	TWI_Write((LCD_ADDR<<1)|0);
	TWI_Write(0x00);
	TWI_Stop();
}

static void LCD_Sleep(void){
	// Turn off display
	LCD_Command(0x08);
	
	// Turn off backlight (send 0x00 to I2C expander)
	TWI_Start();
	TWI_Write((LCD_ADDR<<1)|0);
	TWI_Write(0x00);  // All bits LOW = backlight OFF
	TWI_Stop();
}

static void LCD_Wake(void){
	// Turn on backlight first
	TWI_Start();
	TWI_Write((LCD_ADDR<<1)|0);
	TWI_Write(LCD_BL);  // Backlight ON (0x08)
	TWI_Stop();
	
	// Then turn on display
	LCD_Command(0x0C);  // Display ON, cursor OFF
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

static uint8_t DEC2BCD(uint8_t d){
	return ((d / 10) << 4) | (d % 10);
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

static void DS3231_SetTime(uint8_t h, uint8_t m, uint8_t s){
	TWI_Start();
	TWI_Write((DS3231_ADDR<<1)|0);
	TWI_Write(0x00);
	TWI_Write(DEC2BCD(s));
	TWI_Write(DEC2BCD(m));
	TWI_Write(DEC2BCD(h));
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
	return msb + ((lsb>>6)*0.25f);
}

/* ================= BME280 (simplified) ================= */
#define BME280_REG_ID           0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRESS_MSB    0xF7
#define BME280_REG_CALIB00      0x88
#define BME280_REG_CALIB26      0xE1
#define BME280_CHIP_ID          0x60

typedef struct {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
} BME280_CalibData;

typedef struct {
	float temperature;
	float humidity;
	float pressure;
	float altitude;
	int32_t t_fine;
} BME280_Data;

static BME280_CalibData bme_calib;

static void BME280_WriteReg(uint8_t reg, uint8_t val){
	TWI_Start();
	TWI_Write((BME280_ADDR<<1)|0);
	TWI_Write(reg);
	TWI_Write(val);
	TWI_Stop();
}

static uint8_t BME280_ReadReg(uint8_t reg){
	TWI_Start();
	TWI_Write((BME280_ADDR<<1)|0);
	TWI_Write(reg);
	TWI_Start();
	TWI_Write((BME280_ADDR<<1)|1);
	uint8_t val = TWI_ReadNACK();
	TWI_Stop();
	return val;
}

static void BME280_ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t len){
	TWI_Start();
	TWI_Write((BME280_ADDR<<1)|0);
	TWI_Write(reg);
	TWI_Start();
	TWI_Write((BME280_ADDR<<1)|1);
	for(uint8_t i = 0; i < len-1; i++) buffer[i] = TWI_ReadACK();
	buffer[len-1] = TWI_ReadNACK();
	TWI_Stop();
}

static void BME280_ReadCalibration(void){
	uint8_t calib[26];
	BME280_ReadRegs(BME280_REG_CALIB00, calib, 26);
	bme_calib.dig_T1 = (calib[1] << 8) | calib[0];
	bme_calib.dig_T2 = (calib[3] << 8) | calib[2];
	bme_calib.dig_T3 = (calib[5] << 8) | calib[4];
	bme_calib.dig_P1 = (calib[7] << 8) | calib[6];
	bme_calib.dig_P2 = (calib[9] << 8) | calib[8];
	bme_calib.dig_P3 = (calib[11] << 8) | calib[10];
	bme_calib.dig_P4 = (calib[13] << 8) | calib[12];
	bme_calib.dig_P5 = (calib[15] << 8) | calib[14];
	bme_calib.dig_P6 = (calib[17] << 8) | calib[16];
	bme_calib.dig_P7 = (calib[19] << 8) | calib[18];
	bme_calib.dig_P8 = (calib[21] << 8) | calib[20];
	bme_calib.dig_P9 = (calib[23] << 8) | calib[22];
	bme_calib.dig_H1 = calib[25];
	uint8_t calib_h[7];
	BME280_ReadRegs(BME280_REG_CALIB26, calib_h, 7);
	bme_calib.dig_H2 = (calib_h[1] << 8) | calib_h[0];
	bme_calib.dig_H3 = calib_h[2];
	bme_calib.dig_H4 = (calib_h[3] << 4) | (calib_h[4] & 0x0F);
	bme_calib.dig_H5 = (calib_h[5] << 4) | (calib_h[4] >> 4);
	bme_calib.dig_H6 = calib_h[6];
}

static uint8_t BME280_Init(void){
	_delay_ms(100);
	UART_Print("BME280: Checking chip ID... ");
	uint8_t chip_id = BME280_ReadReg(BME280_REG_ID);
	UART_Print("0x"); 
	char hex[3];
	sprintf(hex, "%02X", chip_id);
	UART_Println(hex);
	if(chip_id != BME280_CHIP_ID){
		UART_Println("ERROR: BME280 not found!");
		return 0;
	}
	BME280_WriteReg(BME280_REG_RESET, 0xB6);
	_delay_ms(100);
	BME280_ReadCalibration();
	BME280_WriteReg(BME280_REG_CTRL_HUM, 0x01);
	_delay_ms(10);
	BME280_WriteReg(BME280_REG_CTRL_MEAS, 0x27);
	_delay_ms(10);
	BME280_WriteReg(BME280_REG_CONFIG, 0xA0);
	_delay_ms(100);
	UART_Println("BME280: OK\n");
	return 1;
}

static int32_t BME280_CompensateTemp(int32_t adc_T, int32_t *t_fine){
	int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme_calib.dig_T1 << 1))) * ((int32_t)bme_calib.dig_T2)) >> 11;
	int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bme_calib.dig_T1))) >> 12) * ((int32_t)bme_calib.dig_T3)) >> 14;
	*t_fine = var1 + var2;
	return (*t_fine * 5 + 128) >> 8;
}

static uint32_t BME280_CompensatePressure(int32_t adc_P, int32_t t_fine){
	int64_t var1 = ((int64_t)t_fine) - 128000;
	int64_t var2 = var1 * var1 * (int64_t)bme_calib.dig_P6;
	var2 = var2 + ((var1 * (int64_t)bme_calib.dig_P5) << 17);
	var2 = var2 + (((int64_t)bme_calib.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)bme_calib.dig_P3) >> 8) + ((var1 * (int64_t)bme_calib.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bme_calib.dig_P1) >> 33;
	if(var1 == 0) return 0;
	int64_t p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)bme_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)bme_calib.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)bme_calib.dig_P7) << 4);
	return (uint32_t)p;
}

static uint32_t BME280_CompensateHumidity(int32_t adc_H, int32_t t_fine){
	int32_t v = (t_fine - ((int32_t)76800));
	v = (((((adc_H << 14) - (((int32_t)bme_calib.dig_H4) << 20) - (((int32_t)bme_calib.dig_H5) * v)) + ((int32_t)16384)) >> 15) *
	    (((((((v * ((int32_t)bme_calib.dig_H6)) >> 10) * (((v * ((int32_t)bme_calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + 
	    ((int32_t)2097152)) * ((int32_t)bme_calib.dig_H2) + 8192) >> 14));
	v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)bme_calib.dig_H1)) >> 4));
	v = (v < 0) ? 0 : v;
	v = (v > 419430400) ? 419430400 : v;
	return (uint32_t)(v >> 12);
}

static void BME280_ReadData(BME280_Data *data){
	uint8_t raw[8];
	BME280_ReadRegs(BME280_REG_PRESS_MSB, raw, 8);
	int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
	int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);
	int32_t adc_H = ((int32_t)raw[6] << 8) | (int32_t)raw[7];
	int32_t t_fine;
	data->temperature = BME280_CompensateTemp(adc_T, &t_fine) / 100.0f;
	data->t_fine = t_fine;
	data->pressure = BME280_CompensatePressure(adc_P, t_fine) / 25600.0f;
	data->humidity = BME280_CompensateHumidity(adc_H, t_fine) / 1024.0f;
	data->altitude = 44330.0f * (1.0f - pow(data->pressure / SEA_LEVEL_PRESSURE, 0.1903f));
}

/* ================= MPU6050 ================= */
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_WHO_AM_I     0x75

typedef struct {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	float temp;
} MPU6050_Data;

static void MPU6050_WriteReg(uint8_t reg, uint8_t val){
	TWI_Start();
	TWI_Write((MPU6050_ADDR<<1)|0);
	TWI_Write(reg);
	TWI_Write(val);
	TWI_Stop();
}

static uint8_t MPU6050_ReadReg(uint8_t reg){
	TWI_Start();
	TWI_Write((MPU6050_ADDR<<1)|0);
	TWI_Write(reg);
	TWI_Start();
	TWI_Write((MPU6050_ADDR<<1)|1);
	uint8_t val = TWI_ReadNACK();
	TWI_Stop();
	return val;
}

static uint8_t MPU6050_Init(void){
	_delay_ms(100);
	uint8_t who = MPU6050_ReadReg(MPU6050_WHO_AM_I);
	UART_Print("MPU6050: WHO_AM_I = 0x");
	char hex[3];
	sprintf(hex, "%02X", who);
	UART_Println(hex);
	if(who != 0x68){
		UART_Println("ERROR: MPU6050 not found!");
		return 0;
	}
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
	_delay_ms(10);
	MPU6050_WriteReg(0x19, 9);
	MPU6050_WriteReg(0x1A, 3);
	MPU6050_WriteReg(0x1B, 0x00);
	MPU6050_WriteReg(0x1C, 0x00);
	_delay_ms(10);
	UART_Println("MPU6050: OK\n");
	return 1;
}

static void MPU6050_ReadData(MPU6050_Data *data){
	TWI_Start();
	TWI_Write((MPU6050_ADDR<<1)|0);
	TWI_Write(MPU6050_ACCEL_XOUT_H);
	TWI_Start();
	TWI_Write((MPU6050_ADDR<<1)|1);
	data->ax = (int16_t)((TWI_ReadACK() << 8) | TWI_ReadACK());
	data->ay = (int16_t)((TWI_ReadACK() << 8) | TWI_ReadACK());
	data->az = (int16_t)((TWI_ReadACK() << 8) | TWI_ReadACK());
	int16_t temp_raw = (int16_t)((TWI_ReadACK() << 8) | TWI_ReadACK());
	data->temp = (temp_raw / 340.0f) + 36.53f;
	data->gx = (int16_t)((TWI_ReadACK() << 8) | TWI_ReadACK());
	data->gy = (int16_t)((TWI_ReadACK() << 8) | TWI_ReadACK());
	data->gz = (int16_t)((TWI_ReadACK() << 8) | TWI_ReadNACK());
	TWI_Stop();
}

/* ================= Motion Detection ================= */
static float filt_ax = 0.0f, filt_ay = 0.0f, filt_az = 0.0f;
static uint8_t filt_initialized = 0;

#define LCD_STATE_NORMAL 0
#define LCD_STATE_DIMMED 1
#define LCD_STATE_SLEEP  2

static uint8_t lcd_state = LCD_STATE_NORMAL;
static uint32_t last_motion_time = 0;

static uint8_t Detect_Motion(MPU6050_Data *data){
	if(!filt_initialized) return 0;
	int16_t diff_x = abs(data->ax - (int16_t)filt_ax);
	int16_t diff_y = abs(data->ay - (int16_t)filt_ay);
	int16_t diff_z = abs(data->az - (int16_t)filt_az);
	return (diff_x > MOTION_THRESHOLD || diff_y > MOTION_THRESHOLD || diff_z > MOTION_THRESHOLD);
}

/* ================= ALARM ================= */
typedef struct {
	uint8_t hour;
	uint8_t minute;
	uint8_t enabled;
	uint8_t ringing;
	uint8_t triggered; // NEW: prevents re-triggering same minute
} Alarm_t;

static Alarm_t alarm = {7, 0, 0, 0, 0};

static void Alarm_Check(uint8_t current_h, uint8_t current_m){
	if(!alarm.enabled) return;
	
	// Check if alarm time matches
	if(current_h == alarm.hour && current_m == alarm.minute){
		if(!alarm.triggered){
			alarm.ringing = 1;
			alarm.triggered = 1;
			UART_Print("?? ALARM TRIGGERED! ");
			UART_PrintInt(current_h);
			UART_Print(":");
			UART_PrintInt(current_m);
			UART_Println("");
		}
	} else {
		// Reset trigger when minute changes
		alarm.triggered = 0;
	}
}

static void Alarm_Stop(void){
	if(alarm.ringing){
		alarm.ringing = 0;
		Buzzer_Off();
		UART_Println("ALARM: STOPPED by user");
	}
}

/* ================= TIMER ================= */
volatile uint32_t stopwatch_time = 0;
volatile uint8_t stopwatch_running = 0;
volatile uint32_t system_ticks = 0;

static void Timer0_Init(void){
	TCCR0A = (1<<WGM01);
	TCCR0B = (1<<CS02) | (1<<CS00);
	OCR0A = 155;
	TIMSK0 = (1<<OCIE0A);
}

ISR(TIMER0_COMPA_vect){
	system_ticks++;
	if(stopwatch_running) stopwatch_time++;
}

/* ================= BUTTONS ================= */
#define BTN_NEXT_PIN     PE4
#define BTN_PREV_PIN     PE5
#define BTN_ACTION_PIN   PD2
#define TOTAL_MENUS 6

volatile uint8_t btn_next_event = 0;
volatile uint8_t btn_prev_event = 0;
volatile uint8_t btn_action_event = 0;

ISR(INT4_vect){ btn_next_event = 1; }
ISR(INT5_vect){ btn_prev_event = 1; }
ISR(INT2_vect){ btn_action_event = 1; }

static void Buttons_Init(void){
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

static void Wake_System(void){
	if(lcd_state != LCD_STATE_NORMAL){
		LCD_Wake();
		lcd_state = LCD_STATE_NORMAL;
		LCD_Clear();
	}
	last_motion_time = system_ticks;
}

/* ================= DISPLAY FUNCTIONS ================= */
static uint8_t menu_index = 0;
static uint8_t stopwatch_state = 0;
static uint8_t mpu_display_mode = 0;
static uint8_t bme_display_mode = 0;
static uint8_t alarm_edit_mode = 0;

static void Stopwatch_Display(void){
	uint32_t t = stopwatch_time;
	uint16_t min = t / 6000;
	uint8_t sec = (t % 6000) / 100;
	uint8_t cs = t % 100;
	
	LCD_SetCursor(0,0);
	if(stopwatch_state == 0) LCD_PrintStr("  STOPWATCH     ");
	else if(stopwatch_state == 1) LCD_PrintStr("  RUNNING...    ");
	else LCD_PrintStr("  PAUSED        ");
	
	LCD_SetCursor(0,1);
	LCD_PrintStr("  ");
	if(min < 10) LCD_Data('0');
	LCD_PrintNum(min & 0xFF);
	LCD_Data(':');
	LCD_PrintNum(sec);
	LCD_Data(':');
	LCD_PrintNum(cs);
	LCD_PrintStr("  ");
}

static void Alarm_Display(void){
	LCD_SetCursor(0,0);
	if(alarm.ringing){
		if((system_ticks % 100) < 50) LCD_PrintStr(">> RINGING!! <<  ");
		else LCD_PrintStr("                ");
	} else if(alarm.enabled){
		LCD_PrintStr("  ALARM ON      ");
	} else {
		LCD_PrintStr("  ALARM OFF     ");
	}
	
	LCD_SetCursor(0,1);
	LCD_PrintStr("   ");
	
	if(alarm_edit_mode == 1 && (system_ticks % 50) < 25) LCD_PrintStr("  ");
	else LCD_PrintNum(alarm.hour);
	
	LCD_Data(':');
	
	if(alarm_edit_mode == 2 && (system_ticks % 50) < 25) LCD_PrintStr("  ");
	else LCD_PrintNum(alarm.minute);
	
	LCD_PrintStr("   ");
}

static void MPU_Display(MPU6050_Data *data){
	char buf[17], tmp1[8], tmp2[8];
	LCD_SetCursor(0,0);

	if(mpu_display_mode == 0){
		LCD_PrintStr("ACCEL m/s2      ");
		LCD_SetCursor(0,1);
		float ax_ms2 = ((filt_initialized ? filt_ax : (float)data->ax) / 16384.0f) * 9.80665f;
		float ay_ms2 = ((filt_initialized ? filt_ay : (float)data->ay) / 16384.0f) * 9.80665f;
		dtostrf(ax_ms2, 5, 1, tmp1);
		dtostrf(ay_ms2, 5, 1, tmp2);
		snprintf(buf, 17, "X%s Y%s      ", tmp1, tmp2);
		LCD_PrintStr(buf);
	}
	else if(mpu_display_mode == 1){
		LCD_PrintStr("GYRO (deg/s)    ");
		LCD_SetCursor(0,1);
		dtostrf(data->gx / 131.0f, 4, 0, tmp1);
		dtostrf(data->gy / 131.0f, 4, 0, tmp2);
		snprintf(buf, 17, "X%s Y%s        ", tmp1, tmp2);
		LCD_PrintStr(buf);
	}
	else {
		LCD_PrintStr("MPU TEMP        ");
		LCD_SetCursor(0,1);
		dtostrf(data->temp, 5, 2, tmp1);
		snprintf(buf, 17, "   %s C        ", tmp1);
		LCD_PrintStr(buf);
	}
}

static void BME_Display(BME280_Data *data){
	char buf[17], tmp[10];
	LCD_SetCursor(0,0);

	if(bme_display_mode == 0){
		LCD_PrintStr("BME TEMP        ");
		LCD_SetCursor(0,1);
		dtostrf(data->temperature, 6, 2, tmp);
		snprintf(buf, 17, "  %s C    ", tmp);
		LCD_PrintStr(buf);
	}
	else if(bme_display_mode == 1){
		LCD_PrintStr("HUMIDITY        ");
		LCD_SetCursor(0,1);
		dtostrf(data->humidity, 5, 1, tmp);
		snprintf(buf, 17, "   %s %%     ", tmp);
		LCD_PrintStr(buf);
	}
	else if(bme_display_mode == 2){
		LCD_PrintStr("PRESSURE        ");
		LCD_SetCursor(0,1);
		dtostrf(data->pressure, 7, 2, tmp);
		snprintf(buf, 17, "%s hPa   ", tmp);
		LCD_PrintStr(buf);
	}
	else {
		LCD_PrintStr("ALTITUDE        ");
		LCD_SetCursor(0,1);
		dtostrf(data->altitude, 7, 1, tmp);
		snprintf(buf, 17, " %s m    ", tmp);
		LCD_PrintStr(buf);
	}
}

static void Handle_Next_Button(void){
	static uint32_t last = 0;
	if((system_ticks - last) < 5) return;
	last = system_ticks;
	
	Wake_System();
	if(alarm.ringing){ Alarm_Stop(); return; }
	
	if(menu_index == 5 && alarm_edit_mode > 0){
		Buzzer_Beep();
		if(alarm_edit_mode == 1) alarm.hour = (alarm.hour + 1) % 24;
		else if(alarm_edit_mode == 2) alarm.minute = (alarm.minute + 1) % 60;
		return;
	}
	
	menu_index = (menu_index + 1) % TOTAL_MENUS;
	LCD_Clear();
}

static void Handle_Prev_Button(void){
	static uint32_t last = 0;
	if((system_ticks - last) < 5) return;
	last = system_ticks;
	
	Wake_System();
	if(alarm.ringing){ Alarm_Stop(); return; }
	
	if(menu_index == 5 && alarm_edit_mode > 0){
		Buzzer_Beep();
		if(alarm_edit_mode == 1) alarm.hour = (alarm.hour == 0) ? 23 : alarm.hour - 1;
		else if(alarm_edit_mode == 2) alarm.minute = (alarm.minute == 0) ? 59 : alarm.minute - 1;
		return;
	}
	
	menu_index = (menu_index == 0) ? TOTAL_MENUS - 1 : menu_index - 1;
	LCD_Clear();
}

/* ================= MAIN ================= */
int main(void){
	UART_Init();
	TWI_Init();
	Timer0_Init();
	LCD_Init();
	Buttons_Init();
	Buzzer_Init();
	
	UART_Println("\n\n?? SMARTWATCH BOOT");
	
	uint8_t mpu_ok = MPU6050_Init();
	uint8_t bme_ok = BME280_Init();
	
	Buzzer_Beep();
	
	uint8_t h, m, s;
	float temp;
	uint8_t last_menu = 0xFF;
	MPU6050_Data mpu_data;
	BME280_Data bme_data;

	LCD_Clear();
	LCD_PrintStr(mpu_ok ? "MPU6050 OK!" : "MPU ERROR!");
	_delay_ms(1000);
	LCD_Clear();
	LCD_PrintStr(bme_ok ? "BME280 OK!" : "BME ERROR!");
	_delay_ms(1000);
	LCD_Clear();
	
	UART_Println("? SYSTEM READY\n");
	
	uint32_t last_mpu_read = 0;
	uint32_t last_bme_read = 0;
	last_motion_time = system_ticks;

	while(1){
		// ?? ALARM RINGING (continuous beeping)
		if(alarm.ringing){
			Buzzer_Alarm_Pattern();
		}
		
		// MPU READ & MOTION DETECT
		if(mpu_ok && (system_ticks - last_mpu_read) >= MPU_READ_TICKS){
			last_mpu_read = system_ticks;
			MPU6050_ReadData(&mpu_data);
			
			if(!filt_initialized){
				filt_ax = (float)mpu_data.ax;
				filt_ay = (float)mpu_data.ay;
				filt_az = (float)mpu_data.az;
				filt_initialized = 1;
			} else {
				filt_ax = EMA_ALPHA * (float)mpu_data.ax + (1.0f - EMA_ALPHA) * filt_ax;
				filt_ay = EMA_ALPHA * (float)mpu_data.ay + (1.0f - EMA_ALPHA) * filt_ay;
				filt_az = EMA_ALPHA * (float)mpu_data.az + (1.0f - EMA_ALPHA) * filt_az;
			}
			
			if(Detect_Motion(&mpu_data)){
				if(lcd_state != LCD_STATE_NORMAL) Wake_System();
				last_motion_time = system_ticks;
			}
		}

		// BME READ
		if(bme_ok && (system_ticks - last_bme_read) >= BME_READ_TICKS){
			last_bme_read = system_ticks;
			BME280_ReadData(&bme_data);
		}

		// CHECK ALARM (every loop cycle!)
		DS3231_GetTime(&h, &m, &s);
		Alarm_Check(h, m);

		// SLEEP MODES
		if(mpu_ok && !alarm.ringing){
			uint32_t idle = system_ticks - last_motion_time;
			if(lcd_state != LCD_STATE_SLEEP && idle >= SLEEP_TIMEOUT_TICKS){
				LCD_Sleep();
				lcd_state = LCD_STATE_SLEEP;
			}
			else if(lcd_state == LCD_STATE_NORMAL && idle >= DIM_TIMEOUT_TICKS){
				LCD_Dim();
				lcd_state = LCD_STATE_DIMMED;
			}
		}

		// BUTTONS
		if(btn_next_event){
			btn_next_event = 0;
			_delay_ms(50);
			if(!(PINE & (1<<BTN_NEXT_PIN))) Handle_Next_Button();
		}
		
		if(btn_prev_event){
			btn_prev_event = 0;
			_delay_ms(50);
			if(!(PINE & (1<<BTN_PREV_PIN))) Handle_Prev_Button();
		}
		
		if(btn_action_event){
			Wake_System();
			btn_action_event = 0;
			_delay_ms(50);
		}

		// STOPWATCH
		if(menu_index == 2){
			static uint8_t pa = 1;
			uint8_t ca = (PIND & (1<<BTN_ACTION_PIN)) ? 1 : 0;
			static uint8_t pressed = 0;
			static uint32_t pstart = 0;
			static uint8_t rdone = 0;
			
			if(pa == 1 && ca == 0){
				pressed = 1;
				pstart = system_ticks;
				rdone = 0;
			}
			
			if(pressed){
				if(!(PIND & (1<<BTN_ACTION_PIN))){
					if((system_ticks - pstart) >= 100 && !rdone){
						stopwatch_time = 0;
						stopwatch_running = 0;
						stopwatch_state = 0;
						rdone = 1;
						Buzzer_Beep();
					}
				} else {
					if((system_ticks - pstart) < 100){
						if(stopwatch_state == 0 || stopwatch_state == 2){
							stopwatch_running = 1;
							stopwatch_state = 1;
						} else {
							stopwatch_running = 0;
							stopwatch_state = 2;
						}
						Buzzer_Beep();
					}
					pressed = 0;
				}
			}
			pa = ca;
		}
		// MPU MODE
		else if(menu_index == 3){
			static uint8_t pa = 1;
			uint8_t ca = (PIND & (1<<BTN_ACTION_PIN)) ? 1 : 0;
			if(pa == 1 && ca == 0){
				mpu_display_mode = (mpu_display_mode + 1) % 3;
				LCD_Clear();
				Buzzer_Beep();
			}
			pa = ca;
		}
		// BME MODE
		else if(menu_index == 4){
			static uint8_t pa = 1;
			uint8_t ca = (PIND & (1<<BTN_ACTION_PIN)) ? 1 : 0;
			if(pa == 1 && ca == 0){
				bme_display_mode = (bme_display_mode + 1) % 4;
				LCD_Clear();
				Buzzer_Beep();
			}
			pa = ca;
		}
		// ALARM MODE
		else if(menu_index == 5){
			static uint8_t pa = 1;
			uint8_t ca = (PIND & (1<<BTN_ACTION_PIN)) ? 1 : 0;
			
			if(pa == 1 && ca == 0){
				Buzzer_Beep();
				if(alarm_edit_mode == 0){
					alarm.enabled = !alarm.enabled;
					UART_Print("Alarm: ");
					UART_Println(alarm.enabled ? "ON" : "OFF");
				} else if(alarm_edit_mode == 1){
					alarm_edit_mode = 2;
				} else if(alarm_edit_mode == 2){
					alarm_edit_mode = 0;
					UART_Print("Alarm set: ");
					UART_PrintInt(alarm.hour);
					UART_Print(":");
					UART_PrintInt(alarm.minute);
					UART_Println("");
				}
			}
			
			static uint32_t hold = 0;
			static uint8_t hdone = 0;
			if(!(PIND & (1<<BTN_ACTION_PIN))){
				if(hold == 0){
					hold = system_ticks;
					hdone = 0;
				}
				if((system_ticks - hold) > 100 && !hdone && alarm_edit_mode == 0){
					alarm_edit_mode = 1;
					hdone = 1;
					Buzzer_Beep();
					UART_Println("Editing alarm hour");
				}
			} else {
				hold = 0;
			}
			pa = ca;
		}

		// DISPLAY UPDATE
		if(lcd_state != LCD_STATE_SLEEP){
			if(last_menu != menu_index){
				last_menu = menu_index;
				if(lcd_state == LCD_STATE_NORMAL) LCD_Clear();
			}

			if(menu_index == 0){
				DS3231_GetTime(&h, &m, &s);
				LCD_SetCursor(0,0);
				LCD_PrintStr("    TIME        ");
				LCD_SetCursor(0,1);
				LCD_PrintStr("   ");
				LCD_PrintNum(h);
				LCD_Data(':');
				LCD_PrintNum(m);
				LCD_Data(':');
				LCD_PrintNum(s);
				LCD_PrintStr("   ");
			} else if(menu_index == 1){
				temp = DS3231_GetTemp();
				LCD_SetCursor(0,0);
				LCD_PrintStr("  RTC TEMP      ");
				LCD_SetCursor(0,1);
				LCD_PrintStr("   ");
				LCD_PrintFloat(temp);
				LCD_Data(0xDF);
				LCD_Data('C');
				LCD_PrintStr("   ");
			} else if(menu_index == 2){
				Stopwatch_Display();
			} else if(menu_index == 3){
				if(mpu_ok) MPU_Display(&mpu_data);
			} else if(menu_index == 4){
				if(bme_ok) BME_Display(&bme_data);
			} else if(menu_index == 5){
				Alarm_Display();
			}
		}

		_delay_ms(50);  // Faster loop for responsive alarm
	}

	return 0;
}
