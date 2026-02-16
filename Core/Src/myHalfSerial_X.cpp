#include "myHalfSerial_X.h"
#include "stm32g0xx_hal_uart.h"
#include <cstring>

myHalfSerial_X::myHalfSerial_X()
    : m_huart(nullptr), m_huart_IT_ready(nullptr), m_isTX(0), m_fifo(nullptr), m_fifo_size(0),
      m_fifo_head(0), m_fifo_tail(0), m_uart_buffer(nullptr), m_uart_buffer_size(0) {}

myHalfSerial_X::~myHalfSerial_X() {
    if (m_fifo) delete[] m_fifo;
    if (m_uart_buffer) delete[] m_uart_buffer;
}

void myHalfSerial_X::init(UART_HandleTypeDef *huart, bool *huart_IT_ready, bool isTX, size_t input_fifo_buffer_size, size_t UART_buffer_size) {
    m_huart = huart;
    m_huart_IT_ready = huart_IT_ready;
	m_isTX = isTX;
    m_fifo_size = input_fifo_buffer_size;
    m_uart_buffer_size = UART_buffer_size;
    m_fifo_head = m_fifo_tail = 0;
    if (m_fifo) delete[] m_fifo;
    if (m_uart_buffer) delete[] m_uart_buffer;
    m_fifo = new uint8_t[m_fifo_size];
    m_uart_buffer = new uint8_t[m_uart_buffer_size];
    if(!m_isTX) {
        *m_huart_IT_ready = false;
        HAL_UART_Receive_IT(m_huart, m_uart_buffer, m_uart_buffer_size);
        HAL_UART_Receive_IT(m_huart, m_uart_buffer, m_uart_buffer_size);
    }
}

int8_t myHalfSerial_X::restart(){
    if(m_isTX) return 0; // No restart needed for TX mode
    m_fifo_head = m_fifo_tail = 0;
    *m_huart_IT_ready = false;
    HAL_UART_Receive_IT(m_huart, m_uart_buffer, m_uart_buffer_size);
    HAL_UART_Receive_IT(m_huart, m_uart_buffer, m_uart_buffer_size);

    return 0;
}
	

size_t myHalfSerial_X::fifo_data_length() const {
    if (m_fifo_head >= m_fifo_tail)
        return m_fifo_head - m_fifo_tail;
    else
        return m_fifo_size - (m_fifo_tail - m_fifo_head);
}

void myHalfSerial_X::fifo_push(uint8_t c) {
    m_fifo[m_fifo_head] = c;
    m_fifo_head = (m_fifo_head + 1) % m_fifo_size;
	if (m_fifo_head == m_fifo_tail) m_fifo_tail=(m_fifo_head+1)%m_fifo_size;
}

uint8_t myHalfSerial_X::fifo_pop() {
    uint8_t c = m_fifo[m_fifo_tail];
    m_fifo_tail = (m_fifo_tail + 1) % m_fifo_size;
    return c;
}

size_t myHalfSerial_X::write(const uint8_t *data_array, size_t len) {
    size_t written = 0;
    size_t free_space = fifo_free_space();
    size_t to_write = (len < free_space) ? len : free_space;
    for (size_t i = 0; i < to_write; ++i) {
        fifo_push(data_array[i]);
        ++written;
    }
    return written;
}

size_t myHalfSerial_X::read( uint8_t *data_array, size_t len) {
    size_t byte_read = 0;
    size_t available_data = fifo_data_length();
    size_t to_read = (len < available_data) ? len : available_data;
    for (size_t i = 0; i < to_read; ++i) {
        data_array[i] = fifo_pop();
        ++	byte_read;
    }
    return byte_read;
}

int8_t myHalfSerial_X::updateSerial() {
	int8_t status=-1;
	if(m_isTX) {
		status=send();
		return status;
	}
	else{
		status=receive();
		return status;
	}
}

int8_t myHalfSerial_X::send() {
    if (!m_huart_IT_ready || !(*m_huart_IT_ready)) //pointer check and ready check
        return -1;
    size_t available = fifo_data_length();
    if (available == 0)
        return 0;
    size_t to_send = (available < m_uart_buffer_size) ? available : m_uart_buffer_size;
    for (size_t i = 0; i < to_send; ++i) {
        m_uart_buffer[i] = fifo_pop();
    }
    *m_huart_IT_ready = false;
    if (HAL_UART_Transmit_IT(m_huart, m_uart_buffer, to_send) == HAL_OK) {
        return (int8_t)to_send;
    } else {
        // On error, push data back to FIFO (not ideal, but prevents loss)
        for (size_t i = 0; i < to_send; ++i) {
            m_fifo_tail = (m_fifo_tail == 0) ? (m_fifo_size - 1) : (m_fifo_tail - 1);
            m_fifo[m_fifo_tail] = m_uart_buffer[to_send - 1 - i];
        }
        *m_huart_IT_ready = true;
        return -1;
    }
}


int8_t myHalfSerial_X::receive() {
    if (!m_huart_IT_ready)    return -1;
	if( !*m_huart_IT_ready)
		return fifo_data_length();
	else{
		for (size_t i = 0; i < m_uart_buffer_size; ++i) {
			fifo_push(m_uart_buffer[i]);
		}
	*m_huart_IT_ready = false;
	HAL_UART_Receive_IT(m_huart, m_uart_buffer,m_uart_buffer_size);
    return fifo_data_length();
	}
}
	

