# CRSF PWM V10 (Private Reimplementation)

This project is a private reimplementation of the Matek CRSF PWM V10 firmware. The target hardware is the Matek CRSF PWM V10 board with its STM32G031 microcontroller. 
Main objective: Learning experience on STM32 HAL programming, improved voltage & current sensor performance vs original code. 
The functionality is a subset of the Matek FW. 
- Configuration from PC is done "in code". 
- No Failsafe output configuration is available (yet)   
The code is developed using STM32CubeMX and Visual Studio Code.

## Overview
- **Platform:** STM32 microcontroller (STM32CubeMX code generation)
- **Development Environment:** Visual Studio Code with STM32CUBE extension + STM32CubeMX

## !!Warning do not use this SW for RC models of any value (this SW is not fit for any such purpose)!! 

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
- A lot of STM32CUBEMX generated files: 
  - `/CRSF_PWM_V10.ioc`: The STM32CubeMX project file is the master for the STM32 code generation. To change or adapt generated code use the STM32CubeMX software
  - `Core/main.c`: No code is written ito this file, just the necessary #includes added plus 2 function calls user_init(), user_loop_step()  
- User files:  
- `Core/src/user_man.cpp`: Arduino like main file conatains "setup()" and "loop()" and tasksk called in these functions
- `Core/src/platform_abstraction.cpp`
  - glue logic to replace the missing arduino plaform functions 
  - adapt to the arduino based AlFredoCRSF library
  - interrupt back functions for the 2 UARTS 
  - DMA call back function for the analog measurements / ADC (2 channels used)  
- `Core/inc/user_main.h`
- `Core/inc/platform_abstraction.h`  



## More Information
For detailed implementation and usage, see comments and documentation within the source code files.
