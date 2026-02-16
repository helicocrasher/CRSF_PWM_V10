#ifndef MYHALFSERIAL_X_H
#define MYHALFSERIAL_X_H

#include "stm32g0xx_hal.h"

class myHalfSerial_X {
public:
    myHalfSerial_X();
    ~myHalfSerial_X();

    void init(UART_HandleTypeDef *huart, bool *huart_IT_ready, bool isTX=true, size_t fifo_buffer_size = 256, size_t UART_buffer_size = 4);
  
    size_t write(const uint8_t *input_array, size_t len);
    int8_t updateSerial();
	size_t read(uint8_t *output_array, size_t len);
	size_t fifo_free_space() const;
    size_t fifo_data_length() const;
	//size_t fifo_free_space(void);
	//size_t fifo_fill_level(void);
	size_t fifo_reset(void);
	int8_t restart();

private:
    UART_HandleTypeDef *m_huart;
    bool *m_huart_IT_ready;
	bool m_isTX;
    uint8_t *m_fifo;
    size_t m_fifo_size;
    size_t m_fifo_head;
    size_t m_fifo_tail;
    uint8_t *m_uart_buffer;
    size_t m_uart_buffer_size;

 //   size_t fifo_free_space() const;
 //   size_t fifo_data_length() const;
    void fifo_push(uint8_t c);
    uint8_t fifo_pop();
    int8_t send();
    int8_t receive();
};

#endif // MYHALFSERIAL_X_H
