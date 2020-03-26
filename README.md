# rbv1.2 implementation

![version](https://img.shields.io/github/v/release/701lab/rbv1.2-implementation?color=blue&label=Version)

This repo can be used as a template for developing applications on robots board v1.2 without deep diving into hardware setups.
Serves three main goals:
1. Provides user with methods to setup all used in device peripherals: system clock, GPIO, timers, etc.
2. Provides user with methods for self-diagnostics of the device and error handling.
3. Provides user with basic data transmitting methods for all interfaces that are used in the design.

**Robots board** v1.2 is a cost affective board to implement differential drive robots, for example, balancing robot. It uses stm32g071cbu MCU, DRV8848 motor driver to control two small  BDC motors with encoders, ICM-20600 3-axis gyroscope and accelerometer, and NRF24l01+ as a radio module. For more information about the board see: [add link to purhase site or/and build files]

## Contains
* implimentation.h and .c files with all implementation dependent methods;
* explanation.h file with implementation decisions explanation;
* main.c file with a minimum amount of code to impleme

## How to use
* clone this repo and add all files into you new robots board v1.2 project;
* go into implementations.h file and set up all user-adjustable defines with respect to the project requirements;
* look through all provided user functions to understand what can be used in the project;
* start development.

### Todo
- [ ] write clean peripherals setup;
- [ ] write error handling;
- [ ] write UART and SPI basic transmission reception functions;
- [ ] write motor control methods;

- [ ] add link to robots board files into readme. 

