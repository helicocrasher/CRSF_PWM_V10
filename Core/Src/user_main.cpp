#include "user_main.h"
#include <cstddef>
#include <cstdint>
#include <stdio.h>
#include "../../AlfredoCRSF/src/AlfredoCRSF.h"
#include "platform_abstraction.h"
#include "stm32g031xx.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_adc.h"

#define CRSF_BATTERY_SENSOR_CELLS_MAX 12
#define BAT_ADC_Oversampling_Ratio 16  // Must match ADC oversampling ratio
#define BAT_ADC_Voltage_divider 11.0f // Voltage divider ratio 10k and 1k resistors
#define StringBufferSize 160


// PWM Channel to Timer and Channel mapping specific for the used "Matek CRSF_PWM_V10" board
TIM_HandleTypeDef* Timer_map[num_PWM_channels]={&htim2,      &htim2,       &htim16,      &htim2,       &htim3,       &htim3,       &htim3,       &htim3,       &htim1,       &htim1};
unsigned int PWM_Channelmap[num_PWM_channels]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_1,TIM_CHANNEL_3,TIM_CHANNEL_4,TIM_CHANNEL_3,TIM_CHANNEL_2,TIM_CHANNEL_1,TIM_CHANNEL_1,TIM_CHANNEL_2};
//   Servo Channel number                             1                 2                 3                 4                 5                 6                 7                 8                 9                 10  
//   Timer channel offset = TIM_Channel_X -1)*4       0                 4                 0                 8                12                 8                 4                 0                 0                  4                

extern "C" {

//user_loop tasks - timed - prototype declarations
static void CRSF_reception_watchdog_task(uint32_t actual_millis);
static void pwm_update_task(uint32_t actual_millis);
static void LED_and_debugSerial_task(uint32_t actual_millis);
static void analog_measurement_task(uint32_t actual_millis);
static void telemetry_transmission_task(uint32_t actual_millis);

static void error_handling_task(void); 

// basic functions
static void sendCellVoltage(uint8_t cellId, float voltage);
static void user_pwm_setvalue(uint8_t pwm_channel, uint16_t PWM_pulse_length);
int8_t send_UART2(const char* msg);

extern ADC_HandleTypeDef hadc1;
//static STM32Stream crsfSerial(&huart1);  // Create STM32Stream object for CRSF communication using UART1
STM32Stream* crsfSerial = nullptr;  // Will be initialized in user_init()
AlfredoCRSF crsf;
volatile uint8_t ready_RX_UART2 = 1;
volatile uint8_t ready_TX_UART2 = 1;
volatile uint8_t ready_RX_UART1 = 1;
volatile uint8_t ready_TX_UART1 = 1;
volatile uint32_t RX1_overrun = 0, crsfSerialRestartRX_counter=0, main_loop_cnt=0, ADC_period=0;
volatile uint32_t ELRS_TX_count = 0, ADC_count=0;
volatile uint16_t ADC_buffer[2]; // ADC buffer for DMA
volatile uint8_t isADCFinished=0;
static float bat_voltage=0.0f, bat_current=0.0f;

/*
// In your initialization (e.g., user_init()):
inline void init_crsf() {
  crsf.begin(*crsfSerial);
    // Now crsf can be used normally
}
*/

void user_init(void)  // same as the "arduino setup()" function
{
  HAL_Delay(5);
  for (unsigned int i=0; i<num_PWM_channels; i++){
    HAL_TIM_PWM_Start(Timer_map[i], PWM_Channelmap[i]);
  }
  // Ensure UART is initialized before creating STM32Stream
  crsfSerial = new STM32Stream(&huart1);
  crsf.begin(*crsfSerial);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(20);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, 2);
}


void user_loop_step(void) // same as the "arduino loop()" function
{
  static uint32_t  actual_millis=0;

  actual_millis = HAL_GetTick();
  crsf.update();
  CRSF_reception_watchdog_task(actual_millis);
  pwm_update_task(actual_millis);
  LED_and_debugSerial_task(actual_millis);
  analog_measurement_task(actual_millis);
  telemetry_transmission_task(actual_millis);
  error_handling_task();
  main_loop_cnt++;
}


static void analog_measurement_task(uint32_t actual_millis) {
  // analog measurement task running with ADC speed is fed by DMA ~22ms with oversampling 16, 
  // ADC clock is 32MHz prescaler 128, sample time 160.5 cycles Total conversion time per channel = 160.5 + 12.5 =173 cycles
  //
  static uint32_t last_adc_millis = 0;
//  static uint16_t adcValue1 =0, adcValue2=0;
  static float adcValue1 =0, adcValue2=0;
  static const float VOLTAGE_DIV = 11.0f;
  static const float SHUNT_RESISTOR_OHMS = 0.066f;
  static const float ADC_GAIN = 1.01f;
  static const float VOLTAGE_OFFSET = 0.00f;
  static const float CURRENT_OFFSET = 0.015f;
  static const float IIR_ALPHA = 0.239057f; // IIR filter alpha coefficient (for Sample_frequency/20 cutoff)
  static const float IIR_BETA = 1.0f - IIR_ALPHA;

  if (isADCFinished == 0) return; // Previous ADC conversion not finished
  ADC_period=actual_millis - last_adc_millis;
  last_adc_millis = actual_millis;
  isADCFinished = 0;
//  adcValue1 = ADC_buffer[0]; // Read the first ADC channel (VBAT);
  adcValue1 = adcValue1*IIR_BETA + (float)ADC_buffer[0] * IIR_ALPHA; // Filter the first ADC channel (VBAT);
  adcValue2 = adcValue2*IIR_BETA + (float)ADC_buffer[1] * IIR_ALPHA; // Filter the second ADC channel (Current);
  bat_voltage = ((float)adcValue1 * 3.3f / 4095.0f * VOLTAGE_DIV * ADC_GAIN/(float)BAT_ADC_Oversampling_Ratio)-VOLTAGE_OFFSET;
  bat_current = ((float)adcValue2 * 3.3f / 4095.0f * ADC_GAIN/(float)BAT_ADC_Oversampling_Ratio / SHUNT_RESISTOR_OHMS)-CURRENT_OFFSET; 
}


/*
static void CRSF_reception_task(void) {
  // Placeholder for future CRSF reception tasks  
}
*/

static void CRSF_reception_watchdog_task(uint32_t actual_millis) {
  // Placeholder for future CRSF reception watchdog tasks  
  static  uint32_t last_watchdog_millis = 0;
  if (crsf.isLinkUp()) {last_watchdog_millis = actual_millis;  return; } // Link is up, reset watchdog timer - nothing to do
  if (actual_millis-last_watchdog_millis <10) return; // Wait for 10ms of no link or last restart attempt
  // No link for more than 10ms - restart CRSF UART RX
  last_watchdog_millis = actual_millis;
  crsfSerial->restartUARTRX(&huart1);
  crsfSerialRestartRX_counter++;
}

static void telemetry_transmission_task(uint32_t actual_millis) {
  // Placeholder for future telemetry transmission tasks  
  static uint32_t last_telemetry_millis = 0;
  static uint32_t telemetry_carousel = 0;
  #define CAROUSEL_MAX 2

  if (actual_millis - last_telemetry_millis < 250/CAROUSEL_MAX) return;
  if(bat_voltage<0.0f) bat_voltage=0.0f;
  if(bat_current<0.0f) bat_current=0.0f;
  if (telemetry_carousel ==0)   sendCellVoltage(1, bat_voltage);
  if (telemetry_carousel ==1)   sendCellVoltage(2, bat_current);
  last_telemetry_millis = actual_millis;
  telemetry_carousel++;
  telemetry_carousel %= CAROUSEL_MAX;
}

static void pwm_update_task(uint32_t actual_millis) {
  // Placeholder for future PWM update tasks 
  static uint32_t servo_update_millis =0; 
  if (actual_millis-servo_update_millis <1) return; // Update every ms to minimize delay between CRSF reception and PWM output
  servo_update_millis = actual_millis;
  for (uint8_t channel=0; channel<num_PWM_channels; channel++){
    uint16_t PWM_value = crsf.getChannel(channel+1);
    user_pwm_setvalue(channel, PWM_value);      
  }
}

static void LED_and_debugSerial_task(uint32_t actual_millis) {
  // Placeholder for future UART communication tasks 
  static uint32_t last_debugTerm_millis=0;
  static char debugMSG[StringBufferSize] = {'\0'};
  uint16_t ch1 = crsf.getChannel(1);
  uint16_t ch2 = crsf.getChannel(2);

  if (actual_millis - last_debugTerm_millis < 200) return;
  last_debugTerm_millis = actual_millis;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14 );
  snprintf(debugMSG, StringBufferSize, "%7lu : ELRS_UP = %1d  / CH1 = %4d CH2 =  %4d, Restart = %4lu ADC_period = %4lu\r\n", (unsigned long) main_loop_cnt, crsf.isLinkUp(), ch1, ch2, (unsigned long)crsfSerialRestartRX_counter,(unsigned long)ADC_period);
  send_UART2(debugMSG);
}

static void error_handling_task(void) {
  // Placeholder for future error handling tasks 
  __NOP();  
}


static void sendCellVoltage(uint8_t cellId, float voltage) {
  static  uint8_t payload[3];
  
  if (cellId < 1 || cellId > CRSF_BATTERY_SENSOR_CELLS_MAX)     return;
  payload[0] = cellId;
  uint16_t voltage_be = htobe16((uint16_t)(voltage * 1000.0)); //mV
  memcpy(&payload[1], &voltage_be, sizeof(voltage_be));
  crsf.queuePacket(CRSF_SYNC_BYTE, 0x0e, payload, sizeof(payload));
}

static void user_pwm_setvalue(uint8_t pwm_channel, uint16_t PWM_pulse_length) {
  static TIM_OC_InitTypeDef sConfigOC={0,0,0,0,0,0,0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  uint32_t count = __HAL_TIM_GET_COUNTER(Timer_map[pwm_channel]);
  if ((count<2250) || (count >19950)) return; // do not update PWM value in the middle of a PWM pulse
  if (PWM_pulse_length <750) PWM_pulse_length =750;
  if (PWM_pulse_length >2250) PWM_pulse_length =2250;
  sConfigOC.Pulse = PWM_pulse_length;
  HAL_TIM_PWM_ConfigChannel(Timer_map[pwm_channel], &sConfigOC, PWM_Channelmap[pwm_channel]);
}

int8_t send_UART2(const char* msg) {   
    if (ready_TX_UART2==0) return -1; // Previous transmission still ongoing
    ready_TX_UART2 = 0; 
    size_t msg_len = strlen(msg);
    if (msg_len > StringBufferSize) {
      msg_len = StringBufferSize; // Limit the message length to prevent overflow
    }
    HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, (uint16_t)msg_len);
    return 0;
}

}  // extern "C"
