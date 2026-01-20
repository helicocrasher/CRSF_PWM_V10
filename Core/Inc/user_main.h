/*
 * user_main.h
 * Extracted user code declarations
 */
#ifndef USER_MAIN_H
#define USER_MAIN_H


#include "main.h"
#include "platform_abstraction.h"
#include <stdint.h>

#define num_PWM_channels 10

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef* Timer_map[num_PWM_channels];
extern unsigned int PWM_Channelmap[num_PWM_channels];


#ifdef __cplusplus
extern "C" {
#endif
void user_init(void);
void user_loop_step(void);
void user_pwm_setvalue(uint8_t pwm_channel, uint16_t PWM_pulse_lengt);
#ifdef __cplusplus
}
#endif

#endif /* USER_MAIN_H */