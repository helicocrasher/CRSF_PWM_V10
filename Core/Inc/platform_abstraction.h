/*
 * platform_abstraction.h
 * Platform abstraction layer for STM32 without Arduino
 * Provides Stream class and millis() for AlfredoCRSF library
 */

#ifndef PLATFORM_ABSTRACTION_H
#define PLATFORM_ABSTRACTION_H

#include "main.h"
#include <stdint.h>
#include <string.h>

void Serial2Debug_print(char* msg) ;

void Serial2Debug_println(char*  msg) ;


#ifdef __cplusplus
// Abstract Stream class for serial communication
class Stream {
public:
    virtual ~Stream() {}
    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t b) = 0;
    virtual size_t write(const uint8_t *buf, size_t len) = 0;
};

// STM32 UART implementation with circular buffer
class STM32Stream : public Stream {
public:
    STM32Stream(UART_HandleTypeDef *huart);
    int restartUARTRX(UART_HandleTypeDef *huart);
    int available() override;
    int read() override;
    size_t write(uint8_t b) override;
    size_t write(const uint8_t *buf, size_t len) override;
    // Called from HAL interrupt callback
    void onRxByte(uint8_t byte);
    // Make these public for C access
    UART_HandleTypeDef *_huart;
    static const uint16_t RX_BUFFER_SIZE = 256;
    uint8_t _rxBuf[256];
    volatile uint16_t _head;
    volatile uint16_t _tail;
};
#endif
// Only visible for C++

#ifdef __cplusplus
class STM32Stream;
extern STM32Stream* g_uartStream;
#endif

// C-compatible helper to re-arm UART RX interrupt
#ifdef __cplusplus
extern "C" {
#endif
void stm32stream_rearm_rx_irq(void);
#ifdef __cplusplus
}
#endif

// Platform-specific millis() implementation
uint32_t platform_millis();

// Helper macro for compatibility
#define millis() platform_millis()


#ifdef __cplusplus
// Arduino-style map() function
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif

#endif /* PLATFORM_ABSTRACTION_H */
