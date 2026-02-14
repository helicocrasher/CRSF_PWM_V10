#include "stm32g0xx_hal.h"


volatile bool ready_TX_UART2 = 1;
UART_HandleTypeDef huart2;
const int FIFOBufferSize = 256;
static char FIFOBuffer[FIFOBufferSize];
static uint16_t FIFOBufferHead = 0;
static uint16_t FIFOBufferTail = 0;

void SerialTX_FIFO_init(){
  for (unsigned int i=0; i<FIFOBufferSize; i++)     FIFOBuffer[i]=0;
}


int writeSerialTXFifoBuffer(const uint8_t* data, volatile size_t length) {
  // Placeholder for future send queue management
  static volatile size_t spaceInFiFo = 0;

  spaceInFiFo =  FIFOBufferSize-(((FIFOBufferHead-FIFOBufferTail) )%FIFOBufferSize);
  if (spaceInFiFo < length) {
    // Not enough space in the buffer, handle overflow (e.g., discard data or reset buffer)
    length = spaceInFiFo;
  }
  for (size_t i = 0; i < length; i++) {
    FIFOBuffer[FIFOBufferHead] = data[i];
    FIFOBufferHead = (FIFOBufferHead + 1) % FIFOBufferSize;
   }
  return length; // Return the number of bytes written to the buffer
}

uint8_t pullFromSerialTXFifoBuffer(char* data, volatile size_t length) {
  // Placeholder for future send queue management
  size_t bytesRead = 0;
  while ((bytesRead < length) && ((FIFOBufferHead - FIFOBufferTail) != 0)) {
    data[bytesRead] = FIFOBuffer[FIFOBufferTail];
    FIFOBufferTail = (FIFOBufferTail + 1) % FIFOBufferSize;
    bytesRead++;
  }
  return bytesRead; // Return the number of bytes read from the buffer
}


int8_t send_UART2(void) { 
  const unsigned int UART2BufferSize = 4;
  static char  UART2Buffer[UART2BufferSize]={"0"};
  static size_t msg_len = 0;

  if (ready_TX_UART2 == 0) return -1; // Previous transmission still in progress
  msg_len = pullFromSerialTXFifoBuffer((char*)UART2Buffer, UART2BufferSize);
  if (msg_len > UART2BufferSize) {
    msg_len = UART2BufferSize; // Limit the message length to prevent overflow
  }
  if (msg_len > 0) {
    ready_TX_UART2 = 0; 
  HAL_UART_Transmit_IT(&huart2, (uint8_t*)UART2Buffer, (uint16_t)msg_len);
  }
  return msg_len;
}
