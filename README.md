# CRSF PWM V10 (Private Reimplementation)

This project is a private reimplementation of the Matek CRSF PWM V10, developed using STM32CubeMX and Visual Studio Code.

## Overview
- **Platform:** STM32 microcontroller (STM32CubeMX code generation)
- **Development Environment:** Visual Studio Code + STM32CubeMX

## Features
- **CRSF Serial Input:** Receives servo data via CRSF protocol
- **CRSF Serial Output:** Sends telemetry data via CRSF protocol
- **10 Servo PWM Outputs:** All channels run at 50Hz
- **Battery Voltage Sensor Input:** Feeds into telemetry
- **Current Sensor Input:** Feeds into telemetry
- **USART2 Debug:** Debug messages are output on USART2
- **Future Extensions:**
  - Barometric sensor data for telemetry
  - GPS data input on USART2 for telemetry

## File Structure
- `Core/` - Main application code (generated and user code)
- `AlfredoCRSF/` - CRSF protocol implementation and examples - fork from https://github.com/AlfredoSystems/AlfredoCRSF
- `Drivers/` - STM32 and CMSIS drivers
- `cmake/` - CMake build configuration

## Files
- A lot of STM32CUBEMX generated files 
- Core/src/user_man.cpp: Arduino like main file conatains "setup()" and "loop()"
- Core/inc/user_main.h :
- Core/src/platform_abstraction.cpp
  - glue logic to replace the missing arduino plaform functions 
  - adapt to the arduino based AlFredoCRSF library
  - interrupt back functions for the 2 UARTS 
  - DMA call back function for the analog measurements / ADC (2 channels used)  



## More Information
For detailed implementation and usage, see comments and documentation within the source code files.
