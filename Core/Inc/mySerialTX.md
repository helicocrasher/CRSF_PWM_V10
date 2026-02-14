# mySerialTX Class for STM32 HAL

## Overview
`mySerialTX` is a non-blocking UART transmission class for STM32 microcontrollers using the HAL library. It leverages `HAL_UART_Transmit_IT` for interrupt-driven transmission and manages an internal FIFO buffer to queue outgoing data. The class is designed for efficient, non-blocking serial output in embedded applications.

## Features
- Non-blocking UART transmission using HAL interrupts
- Internal FIFO buffer to prevent data loss
- Configurable FIFO and UART buffer sizes
- Simple API for writing and sending data
- Prevents buffer overrun by only copying available space

## API

### Constructor / Destructor
```cpp
mySerialTX();
~mySerialTX();
```

### Initialization
```cpp
void init(UART_HandleTypeDef *huart, bool *huart_TX_ready, size_t input_fifo_buffer_size = 256, size_t UART_buffer_size = 4);
```
- `huart`: Pointer to the UART handle
- `huart_TX_ready`: Pointer to a flag indicating UART ready state (should be set to true in the UART Tx complete callback)
- `input_fifo_buffer_size`: Size of the internal FIFO buffer (default 256)
- `UART_buffer_size`: Size of the temporary UART output buffer (default 4)

### Write Data
```cpp
size_t write(const uint8_t *input_array, size_t len);
```
- Copies up to `len` bytes from `input_array` into the FIFO buffer
- Returns the number of bytes actually written (may be less if FIFO is full)

### Send Data
```cpp
int8_t send();
```
- Transfers up to `UART_buffer_size` bytes from FIFO to UART using `HAL_UART_Transmit_IT`
- Returns the number of bytes sent, or -1 if UART is busy

## Usage Example
```cpp
#include "mySerialTX.h"

mySerialTX serialTx;
bool uart_ready = true;

// In initialization code
serialTx.init(&huart2, &uart_ready, 256, 8);

// To write data
uint8_t data[] = "Hello, UART!";
serialTx.write(data, sizeof(data) - 1);

// In main loop or timer
serialTx.send();

// In UART Tx complete callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uart_ready = true;
    }
}
```

## Notes
- The FIFO buffer will not overrun; excess data is not written.
- The `send()` function should be called regularly (e.g., in the main loop or a timer interrupt).
- The `huart_TX_ready` flag must be set to `true` in the UART Tx complete callback.

## License
MIT
