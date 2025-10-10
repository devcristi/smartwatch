# Bare Metal STM-32 Nucleo Based SmartWatch

## Project Overview
This project is a bare-metal implementation of a smartwatch using the STM-32 Nucleo development board. The smartwatch is designed to operate at a low level, utilizing direct register manipulation and avoiding high-level abstractions. The project is written entirely in C.

## Features
- **Real-Time Clock (RTC)**: DS3231 module for precise timekeeping.
- **Motion Sensing**: MPU6050 sensor for accelerometer and gyroscope data.
- **Heart Rate and SpO2 Monitoring**: MAX30102 sensor for health monitoring.
- **Environmental Sensing**: BME280 sensor for temperature, humidity, and pressure measurements.
- **Battery Management**: TP4056 module for battery charging and monitoring.
- **Display**: GC9A01 round LCD for user interface.

## Development Approach
- **Bare-Metal Programming**: The project is implemented without an operating system, focusing on low-level programming.
- **Direct Register Access**: All peripherals are controlled through direct register manipulation, ensuring maximum control and efficiency.
- **Low-Level Drivers**: Custom drivers are developed for each module to interface with the STM-32 microcontroller.

## Hardware Components
1. **STM-32 Nucleo Board**: The core microcontroller platform.
2. **DS3231 RTC Module**: Provides accurate timekeeping.
3. **MPU6050 Sensor**: Measures motion and orientation.
4. **MAX30102 Sensor**: Monitors heart rate and SpO2 levels.
5. **BME280 Sensor**: Captures environmental data such as temperature, humidity, and pressure.
6. **TP4056 Module**: Manages battery charging.
7. **GC9A01 LCD**: Displays smartwatch data and user interface.

## Goals
- Develop a fully functional smartwatch with essential features.
- Implement all functionalities using bare-metal programming techniques.
- Optimize performance and power consumption through efficient low-level coding.

## Future Enhancements
- Add additional sensors or modules for extended functionality.
- Implement power-saving modes to extend battery life.
- Enhance the user interface with advanced graphics.

## Getting Started
1. Clone the repository.
2. Set up the STM-32 Nucleo board with the required hardware components.
3. Compile the code using your preferred ARM compiler.
4. Flash the firmware onto the STM-32 Nucleo board.
5. Connect the hardware modules and power up the system.

## License
This project is open-source and available under the MIT License.