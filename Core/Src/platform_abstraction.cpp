/*
 * platform_abstraction.cpp
 * STM32 implementation of Stream abstraction and timing functions
 */

#include "platform_abstraction.h"

// Global pointer for HAL callback
STM32Stream* g_uartStream = nullptr;

extern UART_HandleTypeDef huart1;

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1 && g_uartStream) {
        // Pass received byte to STM32Stream
        g_uartStream->onRxByte(g_uartStream->_rxBuf[g_uartStream->_head]);
        // Re-arm RX interrupt for next byte
        HAL_UART_Receive_IT(huart, &g_uartStream->_rxBuf[g_uartStream->_head], 5);
    }
}

extern "C" void stm32stream_rearm_rx_irq(void) {
    if (g_uartStream) {
        // Re-enable interrupt for next byte (safe for C)
        HAL_UART_Receive_IT(g_uartStream->_huart, &g_uartStream->_rxBuf[g_uartStream->_head], 1);
    }
}

STM32Stream::STM32Stream(UART_HandleTypeDef *huart) 
    : _huart(huart), _head(0), _tail(0) 
{
    g_uartStream = this;
    // Enable UART RX interrupt
    HAL_UART_Receive_IT(_huart, &_rxBuf[0], 1);
}

int STM32Stream::available() {
    // Calculate available bytes in circular buffer
    uint16_t head = _head;
    uint16_t tail = _tail;
    
    if (head >= tail) {
        return head - tail;
    } else {
        return RX_BUFFER_SIZE - (tail - head);
    }
}

int STM32Stream::read() {
    if (_tail == _head) {
        return -1;  // No data available
    }
    
    uint8_t b = _rxBuf[_tail];
    _tail = (_tail + 1) % RX_BUFFER_SIZE;
    return b;
}

size_t STM32Stream::write(uint8_t b) {
    HAL_UART_Transmit(_huart, &b, 1, 100);
    return 1;
}

size_t STM32Stream::write(const uint8_t *buf, size_t len) {
    HAL_UART_Transmit(_huart, (uint8_t*)buf, len, 100);
    return len;
}

void STM32Stream::onRxByte(uint8_t byte) {
    uint16_t next_head = (_head + 1) % RX_BUFFER_SIZE;
    
    // Prevent buffer overflow - drop oldest byte if buffer is full
    if (next_head != _tail) {
        _rxBuf[_head] = byte;
        _head = next_head;
    }
}

uint32_t platform_millis() {
    return HAL_GetTick();
}
