#include "user_main.h"
#include <stdio.h>
//#include "../AlfredoCRSF/src/AlfredoCRSF.h"
#include "../../AlfredoCRSF/src/AlfredoCRSF.h"
#include "platform_abstraction.h"

#define CRSF_BATTERY_SENSOR_CELLS_MAX 12

TIM_HandleTypeDef* Timer_map[num_PWM_channels];
unsigned int PWM_Channelmap[num_PWM_channels]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_1,TIM_CHANNEL_3,TIM_CHANNEL_4,TIM_CHANNEL_3,TIM_CHANNEL_2,TIM_CHANNEL_1,TIM_CHANNEL_1,TIM_CHANNEL_2};


extern "C" {
static void sendCellVoltage(uint8_t cellId, float voltage);


STM32Stream* crsfSerial = nullptr;  // Will be initialized in user_init()
AlfredoCRSF crsf;
volatile uint8_t ready_RX_UART2 = 1;
volatile uint8_t ready_TX_UART2 = 1;
volatile uint8_t ready_RX_UART1 = 1;
volatile uint8_t ready_TX_UART1 = 1;
volatile uint32_t RX1_overrun = 0, ELRS_TX_count = 0;


// In your initialization (e.g., user_init()):
inline void init_crsf() {
  crsf.begin(crsfSerial);
    // Now crsf can be used normally
}

void user_init(void)
{
  Timer_map[0]=&htim2;
  Timer_map[1]=&htim2;
  Timer_map[2]=&htim16;
  Timer_map[3]=&htim2;
  Timer_map[4]=&htim3;
  Timer_map[5]=&htim3;
  Timer_map[6]=&htim3;
  Timer_map[7]=&htim3;
  Timer_map[8]=&htim1;
  Timer_map[9]=&htim1;

  for (unsigned int i=0; i<num_PWM_channels; i++){
    HAL_TIM_PWM_Start(Timer_map[i], PWM_Channelmap[i]);
  }

  // Ensure UART is initialized before creating STM32Stream
  crsfSerial = new STM32Stream(&huart1);
  crsf.begin(crsfSerial);
}

void user_pwm_setvalue(uint8_t pwm_channel, uint16_t PWM_pulse_lengt)
{
  TIM_OC_InitTypeDef sConfigOC={0,0,0,0,0,0,0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;

  if (PWM_pulse_lengt <750) PWM_pulse_lengt =750;
  if (PWM_pulse_lengt >2250) PWM_pulse_lengt =2250;
  sConfigOC.Pulse = PWM_pulse_lengt;
  HAL_TIM_PWM_ConfigChannel(Timer_map[pwm_channel], &sConfigOC, PWM_Channelmap[pwm_channel]);


}

#define StringBufferSize 60
#define UART2_TX_Buffersize 100

void user_loop_step(void)
{
  static uint32_t  last_250millis=0, servo_update_millis=0;
//  static char MSG[StringBufferSize] = {'\0'};
  static char debugMSG[UART2_TX_Buffersize] = {'\0'};
  static int8_t i=0;
//  static uint8_t up=1;
  static uint16_t loop=0;
  static uint16_t ch1=0, ch2=0;

  uint32_t actual_millis = HAL_GetTick();
  crsf.update();
  if (crsf.isLinkUp()) {
    ch1 = crsf.getChannel(1);  // Get channel 1 in microseconds
    ch2 = crsf.getChannel(2);
  }
  else {ch1 =999; ch2=999;}
  if (actual_millis-servo_update_millis >0) {
    
    servo_update_millis = actual_millis;
        int count = __HAL_TIM_GET_COUNTER(&htim2);
    if ((count>2250) && (count <19950)) {
    for (uint8_t channel=0; channel<num_PWM_channels; channel++){
      uint16_t PWM_value = crsf.getChannel(channel+1);
      user_pwm_setvalue(channel, PWM_value);
    }}
  }
  
  if (actual_millis - last_250millis > 100){
    last_250millis = actual_millis;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14 );
    snprintf(debugMSG, StringBufferSize, "%7d : PWM = %4d ms / CH1 = %4d CH2 =  %4d\r\n", loop, i*4, ch1, ch2);
    send_UART2(debugMSG);
    //HAL_UART_Transmit(&huart2, (const uint8_t *)MSG, sizeof(MSG), 100);
    sendCellVoltage(1, (float) 4.111);
    loop++;
  }


/*
  if(actual_millis - last_millis > 0) {
    int count = __HAL_TIM_GET_COUNTER(&htim2);
    if ((count>2250) && (count <19950)) {
      last_millis = actual_millis;
      for (uint8_t channel=0; channel<num_PWM_channels; channel++){
        uint16_t PWM_value = 1500 + ((int16_t)i*4);
        user_pwm_setvalue(channel, PWM_value);
      }
      if (up==1){
        i++;
        if (i==127) up=0;
      }
      else {
        i--;
        if (i==-127) up=1;
      }
    }
  }
    */
}


int8_t send_UART2(char* msg) {   
    if (ready_TX_UART2==1) {
        ready_TX_UART2 = 0; 
        uint8_t msg_len = strlen(msg);
        if (msg_len > UART2_TX_Buffersize) {
            msg_len = UART2_TX_Buffersize; // Limit the message length to prevent overflow
        }
        HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, msg_len);
        return 0;
    }
    else {
      return -1;
   }
}


void user_HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART2) {
    ready_TX_UART2 = 1;
  } 
  if (huart->Instance == USART1) {
    ready_TX_UART1 = 1; 
  }
}

static void sendCellVoltage(uint8_t cellId, float voltage) {
  if (cellId < 1 || cellId > CRSF_BATTERY_SENSOR_CELLS_MAX)     return;

  uint8_t payload[3];
  payload[0] = cellId;
  uint16_t voltage_be = htobe16((uint16_t)(voltage * 1000.0)); //mV
  memcpy(&payload[1], &voltage_be, sizeof(voltage_be));
  crsf.queuePacket(CRSF_SYNC_BYTE, 0x0e, payload, sizeof(payload));
}

} // extern "C"
