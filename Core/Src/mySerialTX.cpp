#include "mySerialTX.h"
#include <cstring>

mySerialTX::mySerialTX()
    : m_huart(nullptr), m_huart_TX_ready(nullptr), m_fifo(nullptr), m_fifo_size(0),
      m_fifo_head(0), m_fifo_tail(0), m_uart_buffer(nullptr), m_uart_buffer_size(0) {}

mySerialTX::~mySerialTX() {
    if (m_fifo) delete[] m_fifo;
    if (m_uart_buffer) delete[] m_uart_buffer;
}

void mySerialTX::init(UART_HandleTypeDef *huart, bool *huart_TX_ready, size_t input_fifo_buffer_size, size_t UART_buffer_size) {
    m_huart = huart;
    m_huart_TX_ready = huart_TX_ready;
    m_fifo_size = input_fifo_buffer_size;
    m_uart_buffer_size = UART_buffer_size;
    m_fifo_head = m_fifo_tail = 0;
    if (m_fifo) delete[] m_fifo;
    if (m_uart_buffer) delete[] m_uart_buffer;
    m_fifo = new uint8_t[m_fifo_size];
    m_uart_buffer = new uint8_t[m_uart_buffer_size];
}

size_t mySerialTX::fifo_free_space() const {
    if (m_fifo_head >= m_fifo_tail)
        return m_fifo_size - (m_fifo_head - m_fifo_tail) - 1;
    else
        return m_fifo_tail - m_fifo_head - 1;
}

size_t mySerialTX::fifo_data_length() const {
    if (m_fifo_head >= m_fifo_tail)
        return m_fifo_head - m_fifo_tail;
    else
        return m_fifo_size - (m_fifo_tail - m_fifo_head);
}

void mySerialTX::fifo_push(uint8_t c) {
    m_fifo[m_fifo_head] = c;
    m_fifo_head = (m_fifo_head + 1) % m_fifo_size;
}

uint8_t mySerialTX::fifo_pop() {
    uint8_t c = m_fifo[m_fifo_tail];
    m_fifo_tail = (m_fifo_tail + 1) % m_fifo_size;
    return c;
}

size_t mySerialTX::write(const uint8_t *input_array, size_t len) {
    size_t written = 0;
    size_t free_space = fifo_free_space();
    size_t to_write = (len < free_space) ? len : free_space;
    for (size_t i = 0; i < to_write; ++i) {
        fifo_push(input_array[i]);
        ++written;
    }
    return written;
}

int8_t mySerialTX::send() {
    if (!m_huart_TX_ready || !(*m_huart_TX_ready))
        return -1;
    size_t available = fifo_data_length();
    if (available == 0)
        return 0;
    size_t to_send = (available < m_uart_buffer_size) ? available : m_uart_buffer_size;
    for (size_t i = 0; i < to_send; ++i) {
        m_uart_buffer[i] = fifo_pop();
    }
    *m_huart_TX_ready = false;
    if (HAL_UART_Transmit_IT(m_huart, m_uart_buffer, to_send) == HAL_OK) {
        return (int8_t)to_send;
    } else {
        // On error, push data back to FIFO (not ideal, but prevents loss)
        for (size_t i = 0; i < to_send; ++i) {
            m_fifo_tail = (m_fifo_tail == 0) ? (m_fifo_size - 1) : (m_fifo_tail - 1);
            m_fifo[m_fifo_tail] = m_uart_buffer[to_send - 1 - i];
        }
        *m_huart_TX_ready = true;
        return -1;
    }
}
