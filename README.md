# Bare Metal STM-32 Nucleo Based SmartWatch# Bare Metal STM-32 Nucleo Based SmartWatch



## Project Overview## Project Overview

This project is a bare-metal implementation of a smartwatch using an AVR microcontroller (ATmega2560). The smartwatch is designed to operate at a low level, utilizing direct register manipulation and avoiding high-level abstractions. The entire project is written in C with modular architecture.This project is a bare-metal implementation of a smartwatch using the STM-32 Nucleo development board. The smartwatch is designed to operate at a low level, utilizing direct register manipulation and avoiding high-level abstractions. The project is written entirely in C.



## Features## Features

- **Real-Time Clock (RTC)**: DS3231 module for precise timekeeping with I2C interface- **Real-Time Clock (RTC)**: DS3231 module for precise timekeeping.

- **LCD Display**: GC9A01 compatible 16x2 LCD via I2C- **Motion Sensing**: MPU6050 sensor for accelerometer and gyroscope data.

- **Menu Navigation**: 3-mode menu system (Time, Temperature, Stopwatch)- **Heart Rate and SpO2 Monitoring**: MAX30102 sensor for health monitoring.

- **Stopwatch Functionality**: Start/Pause and Reset operations with centisecond precision- **Environmental Sensing**: BME280 sensor for temperature, humidity, and pressure measurements.

- **Temperature Monitoring**: Real-time temperature reading from DS3231 sensor- **Battery Management**: TP4056 module for battery charging and monitoring.

- **Button Control**: Three interrupt-driven buttons for navigation and control- **Display**: GC9A01 round LCD for user interface.

- **Timer-Based Scheduling**: TIMER1 for precise timing and interrupt handling

## Development Approach

## Hardware Components- **Bare-Metal Programming**: The project is implemented without an operating system, focusing on low-level programming.

1. **ATmega2560 Microcontroller**: Core processor- **Direct Register Access**: All peripherals are controlled through direct register manipulation, ensuring maximum control and efficiency.

2. **DS3231 RTC Module**: Real-time clock and temperature sensor (I2C @ 0x68)- **Low-Level Drivers**: Custom drivers are developed for each module to interface with the STM-32 microcontroller.

3. **LCD 16x2 Module**: Display with I2C backpack (I2C @ 0x27)

4. **Push Buttons**: ## Hardware Components

   - Button NEXT (PE4, INT4): Navigate menu forward1. **STM-32 Nucleo Board**: The core microcontroller platform.

   - Button PREV (PE5, INT5): Navigate menu backward  2. **DS3231 RTC Module**: Provides accurate timekeeping.

   - Button ACTION (PD2, INT2): Start/Pause/Reset stopwatch3. **MPU6050 Sensor**: Measures motion and orientation.

4. **MAX30102 Sensor**: Monitors heart rate and SpO2 levels.

## Project Structure5. **BME280 Sensor**: Captures environmental data such as temperature, humidity, and pressure.

```6. **TP4056 Module**: Manages battery charging.

smartwatch/7. **GC9A01 LCD**: Displays smartwatch data and user interface.

├── Core/

│   ├── Inc/                    # Header files## Goals

│   │   ├── main.h              # Main application header- Develop a fully functional smartwatch with essential features.

│   │   ├── i2c.h               # I2C/TWI interface- Implement all functionalities using bare-metal programming techniques.

│   │   ├── buttons.h           # Button handling- Optimize performance and power consumption through efficient low-level coding.

│   │   ├── timer.h             # Timer interrupt handling

│   │   └── stopwatch.h         # Stopwatch interface## Future Enhancements

│   └── Src/                    # Source files- Add additional sensors or modules for extended functionality.

│       ├── main.c              # Main application logic- Implement power-saving modes to extend battery life.

│       ├── i2c.c               # I2C driver implementation- Enhance the user interface with advanced graphics.

│       ├── buttons.c           # Button ISR and handling

│       ├── timer.c             # Timer ISR implementation## Getting Started

│       └── stopwatch.c         # Stopwatch logic1. Clone the repository.

├── Drivers/2. Set up the STM-32 Nucleo board with the required hardware components.

│   └── Custom/3. Compile the code using your preferred ARM compiler.

│       ├── Inc/4. Flash the firmware onto the STM-32 Nucleo board.

│       │   ├── lcd_i2c.h       # LCD I2C driver header5. Connect the hardware modules and power up the system.

│       │   └── ds3231.h        # DS3231 RTC driver header

│       └── Src/## License

│           ├── lcd_i2c.c       # LCD I2C driver implementationThis project is open-source and available under the MIT License.
│           └── ds3231.c        # DS3231 RTC driver implementation
└── README.md                   # This file
```

## Development Approach
- **Bare-Metal Programming**: No OS, direct hardware register access
- **Direct Register Manipulation**: Full control over microcontroller peripherals
- **Modular Architecture**: Separated concerns for easy maintenance and extension
- **ISR-Driven**: Interrupt Service Routines for button inputs and timing
- **I2C Communication**: Two-wire interface for RTC and LCD modules

## Functional Modes

### Mode 1: TIME Display
- Shows current time from DS3231 RTC
- Format: HH:MM:SS
- Hold PREV button (>1s) to reset clock to 00:00:00

### Mode 2: TEMPERATURE Display
- Shows current temperature from DS3231 sensor
- Format: XX.XX°C (with 0.25°C precision)

### Mode 3: STOPWATCH
- Start/Pause: Short press of ACTION button
- Reset: Long press (>1s) of ACTION button
- Format: MM:SS:CS (minutes:seconds:centiseconds)

## Building the Project

### Prerequisites
- ARM GCC compiler for AVR (avr-gcc)
- avrdude for flashing
- Make (optional, for automated builds)

### Compilation
```bash
avr-gcc -mmcu=atmega2560 -DF_CPU=16000000UL -O2 \
  Core/Src/*.c Drivers/Custom/Src/*.c \
  -o smartwatch.elf

avr-objcopy -O ihex smartwatch.elf smartwatch.hex
```

### Flashing
```bash
avrdude -p m2560 -c stk500v2 -P COM3 -b 115200 -U flash:w:smartwatch.hex:i
```

## Pin Configuration

| Function | Pin | AVR Port | Interrupt |
|----------|-----|----------|-----------|
| Button NEXT | PE4 | PORTE4 | INT4 |
| Button PREV | PE5 | PORTE5 | INT5 |
| Button ACTION | PD2 | PORTD2 | INT2 |
| I2C SDA | PD1 | PORTD1 | - |
| I2C SCL | PD0 | PORTD0 | - |

## I2C Configuration
- **Speed**: 100 kHz (TWBR = 72)
- **Devices**:
  - DS3231 RTC: Address 0x68
  - LCD Module: Address 0x27

## Timer Configuration
- **Timer1**: 16-bit CTC mode
- **Prescaler**: 256 (CS12 = 1)
- **Compare Register**: 624 (generates 10ms interrupt at 16MHz)
- **Frequency**: 100 Hz system ticks

## Interrupt Service Routines
1. **INT4_vect**: NEXT button handler
2. **INT5_vect**: PREV button handler
3. **INT2_vect**: ACTION button handler
4. **TIMER1_COMPA_vect**: Timer compare interrupt for timing

## Key Features Implementation

### Debouncing
- 50ms software debounce on button presses
- Prevents false triggers from mechanical contact bounce

### Time Management
- System ticks every 10ms (from TIMER1)
- Stopwatch counts in 10ms increments
- Button press timing for long-press detection

### I2C Communication
- Blocking I2C transfers (appropriate for low-speed communication)
- Standard START/STOP conditions
- ACK/NACK handling for multi-byte reads

## Future Enhancements
- Add additional sensors (accelerometer MPU6050, heart rate MAX30102)
- Implement BME280 environmental sensor
- Add TP4056 battery charging management
- Power management and sleep modes
- More sophisticated menu system with settings
- Data logging capabilities

## Troubleshooting

### LCD Not Displaying
1. Check I2C address (0x27) with I2C scanner
2. Verify pull-up resistors on SDA/SCL lines
3. Check contrast adjustment on LCD module

### RTC Time Not Updating
1. Verify DS3231 I2C address (0x68)
2. Check if coin cell battery is installed
3. Ensure I2C communication is working

### Buttons Not Responding
1. Check pin configuration matches header defines
2. Verify pull-up resistors are present
3. Test with multimeter for continuity

## License
This project is open-source and available under the MIT License.

## Author
devcristi - Embedded Systems Development

## References
- AVR ATmega2560 Datasheet
- DS3231 RTC Datasheet
- I2C/TWI Communication Protocol
