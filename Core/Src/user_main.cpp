#include "user_main.h"
#include <stdio.h>

TIM_HandleTypeDef* Timer_map[num_PWM_channels];
unsigned int PWM_Channelmap[num_PWM_channels]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_1,TIM_CHANNEL_3,TIM_CHANNEL_4,TIM_CHANNEL_3,TIM_CHANNEL_2,TIM_CHANNEL_1,TIM_CHANNEL_1,TIM_CHANNEL_2};

extern "C" {

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

void user_loop_step(void)
{
  static uint32_t last_millis=0, last_250millis=0;
  static char MSG[40] = {'\0'};
  static int8_t i=0;
  static uint8_t up=1;
  static uint16_t loop=0;

  uint32_t actual_millis = HAL_GetTick();
  if (actual_millis - last_250millis > 100){
    last_250millis = actual_millis;
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14 );
    sprintf(MSG, "%7d : PWM = %4d ms\r\n", loop, i*4);
    HAL_UART_Transmit(&huart2, (const uint8_t *)MSG, sizeof(MSG), 100);
    loop++;
  }

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
}

} // extern "C"
