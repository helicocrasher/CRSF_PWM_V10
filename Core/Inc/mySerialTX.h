#ifndef MYSERIALTX_H
#define MYSERIALTX_H

#include "stm32g0xx_hal.h"

class mySerialTX {
public:
    mySerialTX();
    ~mySerialTX();

    void init(UART_HandleTypeDef *huart, bool *huart_TX_ready, size_t input_fifo_buffer_size = 256, size_t UART_buffer_size = 4);
    size_t write(const uint8_t *input_array, size_t len);
    int8_t send();

private:
    UART_HandleTypeDef *m_huart;
    bool *m_huart_TX_ready;
    uint8_t *m_fifo;
    size_t m_fifo_size;
    size_t m_fifo_head;
    size_t m_fifo_tail;
    uint8_t *m_uart_buffer;
    size_t m_uart_buffer_size;

    size_t fifo_free_space() const;
    size_t fifo_data_length() const;
    void fifo_push(uint8_t c);
    uint8_t fifo_pop();
};

#endif // MYSERIALTX_H
