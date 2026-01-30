/*// Arduino-style map() function implementation (for C++ linkage)
#ifdef __cplusplus
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif
*/
/*
 * platform_abstraction.cpp
 * STM32 implementation of Stream abstraction and timing functions
 */

#include "platform_abstraction.h"
#include "stm32g0xx_hal_adc.h"
#include "stm32g0xx_hal_uart.h"
#include <cstdint>
#include <cstring>
#include <sys/_intsup.h>

// Global pointer for HAL callback
STM32Stream* g_uartStream = nullptr;

extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern volatile uint32_t RX1_overrun, ELRS_TX_count;
extern volatile uint8_t ready_RX_UART1;
extern volatile uint8_t ready_TX_UART1;
extern volatile uint8_t ready_TX_UART2; 
extern volatile uint8_t ready_RX_UART2; 
extern volatile uint32_t adcValue, ADC_count;
extern int isADCFinished;

char UART1_TX_Buffer[20];

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        // Handle ADC conversion complete event
        // For example, read the converted value
        // adcValue = HAL_ADC_GetValue(&hadc1);
        // Process adcValue as needed
        ADC_count++;
        isADCFinished = 1;
    }
//    while (1);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
    ready_TX_UART2 = 1;
  } 
  if (huart->Instance == USART1) {
    ready_TX_UART1 = 1; 
  }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1 && g_uartStream) {
        // Pass received byte to STM32Stream
        if (huart->ErrorCode == HAL_UART_ERROR_ORE) {
            // Handle overrun error
            __HAL_UART_CLEAR_OREFLAG(&huart1);
        RX1_overrun++;      
    }
        g_uartStream->onRxByte(g_uartStream->_rxBuf[g_uartStream->_head]);
        // Re-arm RX interrupt for next byte
        HAL_UART_Receive_IT(huart, &g_uartStream->_rxBuf[g_uartStream->_head], 1);
    }
}
/*
extern "C" void stm32stream_rearm_rx_irq(void) {
    if (g_uartStream) {
        // Re-enable interrupt for next byte (safe for C)
        HAL_UART_Receive_IT(g_uartStream->_huart, &g_uartStream->_rxBuf[g_uartStream->_head], 1);
    }
}
*/

STM32Stream::STM32Stream(UART_HandleTypeDef *huart) 
    : _huart(huart), _head(0), _tail(0) 
{
    if (huart == &huart1 && g_uartStream) {
        // Pass received byte to STM32Stream
        if (huart->ErrorCode == HAL_UART_ERROR_ORE) {
            // Handle overrun error
            __HAL_UART_CLEAR_OREFLAG(&huart1);
            RX1_overrun++; 
        }
    }
    g_uartStream = this;
    // Enable UART RX interrupt
    HAL_UART_Receive_IT(_huart, &_rxBuf[0], 1); // 2 calls required to reliably start CRSF UART RX
    HAL_UART_Receive_IT(_huart, &_rxBuf[0], 1);
}

int STM32Stream::restartUARTRX(UART_HandleTypeDef *huart) {
    _huart = huart;
    _head = 0;
    _tail = 0;
    // Enable UART RX interrupt
    HAL_UART_Receive_IT(_huart, &_rxBuf[0], 1); // 2 calls required to reliably restart CRSF UART RX
    HAL_UART_Receive_IT(_huart, &_rxBuf[0], 1);
    return 0;
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
//    HAL_UART_Transmit(_huart, &b, 1, 100); // Blocking version
    if (ready_TX_UART1==  1 ) { // Non-blocking version - not tested
      ELRS_TX_count+=1;  
      memcpy(&UART1_TX_Buffer[0], &b, 1);
      HAL_UART_Transmit_IT(_huart, (uint8_t*)UART1_TX_Buffer, 1); 
      ready_TX_UART1 = 0;
      return 1;
    }
    else {
        return 0;
    }
}

size_t STM32Stream::write(const uint8_t *buf, size_t len) {

//    HAL_UART_Transmit(_huart, (uint8_t*)buf, len, 100); // Blocking version - does work but blocking
    if (ready_TX_UART1==  1 ) { // Non-blocking version - does work
      ELRS_TX_count+=len;  
      memcpy(&UART1_TX_Buffer[0], buf, len);
      HAL_UART_Transmit_IT(_huart, (uint8_t*)UART1_TX_Buffer, len); 
      ready_TX_UART1 = 0;
      return len;
    }
    else {
        return 0;
    }
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
