#include "user_main.h"
#include <cstddef>
#include <cstdint>
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include "../../AlfredoCRSF/src/AlfredoCRSF.h"
#include "platform_abstraction.h"
#include "stm32g031xx.h"
#include "stm32g0xx_hal.h"

#include "stm32g0xx_hal_adc.h"
#include "../../SPL06-001/SPL06-001.h"
#include "../../SPL06-001/spl06_001_glue.h" // For SerialI2CDebug macros
#include "mySerialTX.h"

#define CRSF_BATTERY_SENSOR_CELLS_MAX 12
#define BAT_ADC_Oversampling_Ratio 16  // Must match ADC oversampling ratio
#define BAT_ADC_Voltage_divider 11.0f // Voltage divider ratio 10k and 1k resistors
#define DBG_STRINGBUFSIZE 100
char debug_str_buffer[DBG_STRINGBUFSIZE];

// PWM Channel to Timer and Channel mapping specific for the used "Matek CRSF_PWM_V10" board
TIM_HandleTypeDef* Timer_map[num_PWM_channels]={&htim2,      &htim2,       &htim16,      &htim2,       &htim3,       &htim3,       &htim3,       &htim3,       &htim1,       &htim1};
unsigned int PWM_Channelmap[num_PWM_channels]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_1,TIM_CHANNEL_3,TIM_CHANNEL_4,TIM_CHANNEL_3,TIM_CHANNEL_2,TIM_CHANNEL_1,TIM_CHANNEL_1,TIM_CHANNEL_2};
//   Servo Channel number                             1                 2                 3                 4                 5                 6                 7                 8                 9                 10  
//   Timer channel offset = TIM_Channel_X -1)*4       0                 4                 0                 8                12                 8                 4                 0                 0                  4                

extern "C" {

#define TARGET_MATEKSYS_CRSF_PWM_V10
TwoWire BaroWire(SDA2, SCL2);	
SPL06 BaroSensor(&BaroWire); 
mySerialTX serial2TX;
void setupBaroSensor();  
void baroProcessingTask(uint32_t millis_now);
void baroSerialDisplayTask(uint32_t millis_now);
void telemetrySendBaroAltitude(float altitude);
void telemetrySendVario( float verticalspd);


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
int8_t send_UART2(void);

extern ADC_HandleTypeDef hadc1;
//static STM32Stream crsfSerial(&huart1);  // Create STM32Stream object for CRSF communication using UART1
STM32Stream* crsfSerial = nullptr;  // Will be initialized in user_init()
AlfredoCRSF crsf;
volatile uint8_t ready_RX_UART2 = 1;
volatile bool ready_TX_UART2 = 1;
volatile uint8_t ready_RX_UART1 = 1;
volatile bool ready_TX_UART1 = 1;
volatile uint32_t RX1_overrun = 0, crsfSerialRestartRX_counter=0, main_loop_cnt=0, ADC_period=0;
volatile uint32_t ELRS_TX_count = 0, ADC_count=0;
volatile uint16_t ADC_buffer[2]; // ADC buffer for DMA
volatile uint8_t isADCFinished=0;
volatile uint8_t i2cWriteComplete=1;
static float bat_voltage=0.0f, bat_current=0.0f;

static const float IIR_ALPHA  = 0.135755f;  // 0.5Hz cut off (fc/40 / fc=Hz /Tc=320ms)
static const float IIR_BETA  = (1.0f - IIR_ALPHA);

static float GND_altitude=0;
static float previous_alt_ASL=0, baroAltitude=0, baroTemperature=0, baroPressure=0;
static float vario=0,filt_alt_AGL=0;
static double filt_vario=0, filt_alt_ASL=0;
static uint32_t GND_alt_count=0;

const int RingBufferSize = 256;
static char RingBuffer[RingBufferSize];
static uint16_t RingBufferHead = 0;
static uint16_t RingBufferTail = 0;

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
  serial2TX.init(&huart2, (bool*)&ready_TX_UART2, 256, 4  );
  for (unsigned int i=0; i<256; i++)     RingBuffer[i]=0;
  for (unsigned int i=0; i<num_PWM_channels; i++){
    HAL_TIM_PWM_Start(Timer_map[i], PWM_Channelmap[i]);
  }
  // Ensure UART is initialized before creating STM32Stream
  crsfSerial = new STM32Stream(&huart1);
  crsf.begin(*crsfSerial);
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(20);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_buffer, 2);
  setupBaroSensor();
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
  baroProcessingTask(actual_millis);
  baroSerialDisplayTask(actual_millis);
  telemetry_transmission_task(actual_millis);
  error_handling_task();
  send_UART2();
  serial2TX.send();
  main_loop_cnt++;
  HAL_Delay(2)  ; // Small delay to prevent CPU hogging - adjust as needed for timing

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

  static  uint32_t last_watchdog_millis = 0;
  if (crsf.isLinkUp()) {last_watchdog_millis = actual_millis;  return; } // Link is up, reset watchdog timer - nothing to do
  if (actual_millis-last_watchdog_millis <10) return; // Wait for 10ms of no link or last restart attempt
  // No link for more than 10ms - restart CRSF UART RX
  last_watchdog_millis = actual_millis;
  crsfSerial->restartUARTRX(&huart1);
  crsfSerialRestartRX_counter++;
}



int writeSerialTXFifoBuffer(const uint8_t* data, volatile size_t length) {
  // Placeholder for future send queue management
  static volatile size_t spaceInFiFo = 0;

  spaceInFiFo =  RingBufferSize-(((RingBufferHead-RingBufferTail) )%RingBufferSize);
  if (spaceInFiFo < length) {
    // Not enough space in the buffer, handle overflow (e.g., discard data or reset buffer)
    length = spaceInFiFo;
  }
  for (size_t i = 0; i < length; i++) {
    RingBuffer[RingBufferHead] = data[i];
    RingBufferHead = (RingBufferHead + 1) % RingBufferSize;
   }
  return length; // Return the number of bytes written to the buffer
}

uint8_t pullFromSerialTXFifoBuffer(char* data, volatile size_t length) {
  // Placeholder for future send queue management
  size_t bytesRead = 0;
  while ((bytesRead < length) && ((RingBufferHead - RingBufferTail) != 0)) {
    data[bytesRead] = RingBuffer[RingBufferTail];
    RingBufferTail = (RingBufferTail + 1) % RingBufferSize;
    bytesRead++;
  }
  return bytesRead; // Return the number of bytes read from the buffer
}

static void telemetry_transmission_task(uint32_t actual_millis) {
  static uint32_t last_telemetry_millis = 0;
  static uint32_t telemetry_carousel = 0;
  #define CAROUSEL_MAX 4

  if (actual_millis - last_telemetry_millis < 500/CAROUSEL_MAX) return;
  if(bat_voltage<0.0f) bat_voltage=0.0f;
  if(bat_current<0.0f) bat_current=0.0f;
  if (telemetry_carousel ==0)   sendCellVoltage(1, bat_voltage);
  if (telemetry_carousel ==1)   sendCellVoltage(2, bat_current);
  if (telemetry_carousel ==2)   telemetrySendBaroAltitude(filt_alt_AGL);
  if (telemetry_carousel ==3)   telemetrySendVario( filt_vario);
  last_telemetry_millis = actual_millis;
  telemetry_carousel++;
  telemetry_carousel %= CAROUSEL_MAX;
}

static void pwm_update_task(uint32_t actual_millis) {

  static uint32_t servo_update_millis =0; 
  if (actual_millis-servo_update_millis <1) return; // Update every ms to minimize delay between CRSF reception and PWM output
  servo_update_millis = actual_millis;
  for (uint8_t channel=0; channel<num_PWM_channels; channel++){
    uint16_t PWM_value = crsf.getChannel(channel+1);
    user_pwm_setvalue(channel, PWM_value);      
  }
}

static void LED_and_debugSerial_task(uint32_t actual_millis) {

  static uint32_t last_debugTerm_millis=0;
  uint16_t ch1 = crsf.getChannel(1);
  uint16_t ch2 = crsf.getChannel(2);

  if (actual_millis - last_debugTerm_millis < 200) return;
  last_debugTerm_millis = actual_millis;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14 );
  snprintf(debug_str_buffer, DBG_STRINGBUFSIZE, "%7lu : ELRS_UP = %1d  / CH1 = %4d CH2 =  %4d, Restart = %4lu ADC_period = %4lu\r\n", (unsigned long) main_loop_cnt, crsf.isLinkUp(), ch1, ch2, (unsigned long)crsfSerialRestartRX_counter,(unsigned long)ADC_period);
  serial2TX.write((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
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


void setupBaroSensor(){   // SPL06-001 sensor version 
  unsigned status;

  BaroWire.begin();

// uint32_t read_data;
// getI2cData(SPL06_ADDRESS_ALT, 0x0d, 1, (short*) &read_data);

  HAL_Delay(10);
  
  status = BaroSensor.begin(SPL06_ADDRESS_ALT,SPL06_PRODID);
  //unsigned status = BaroSensor.begin(SPL06_ADDRESS_ALT);
  if (!status) {
    snprintf(debug_str_buffer, sizeof(debug_str_buffer), "Could not find a valid SPL06-001 sensor, check wiring or try a different address!\\n\r");
    serial2TX.write((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
    {
      
      snprintf(debug_str_buffer, sizeof(debug_str_buffer), "SensorID was:   0x%02X", BaroSensor.sensorID());
      serial2TX.write((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
    }
    snprintf(debug_str_buffer, sizeof(debug_str_buffer), "        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
    serial2TX.write((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
    while (1) HAL_Delay(10);
  }
  BaroSensor.setSampling(SPL06::MODE_BACKGND_BOTH,
                          SPL06::SAMPLING_X16,
                          SPL06::SAMPLING_X16,
                          SPL06::RATE_X16,
                          SPL06::RATE_X16);
  snprintf(debug_str_buffer, sizeof(debug_str_buffer), "SPL06-001 ready");
//  writeSerialTXFifoBuffer((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
  serial2TX.write((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));  
}


void telemetrySendBaroAltitude(float altitude)
{
  crsf_sensor_baro_altitude_t crsfBaroAltitude = { 0 , 0 };

  // Values are MSB first (BigEndian)
  crsfBaroAltitude.altitude = htobe16((uint16_t)(altitude*10.0 + 10000.0));
  //crsfBaroAltitude.verticalspd = htobe16((int16_t)(verticalspd*100.0)); //TODO: fix verticalspd in BaroAlt packets
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BARO_ALTITUDE, &crsfBaroAltitude, sizeof(crsfBaroAltitude) - 2);
  
  //Supposedly vertical speed can be sent in a BaroAltitude packet, but I cant get this to work.
  //For now I have to send a second vario packet to get vertical speed telemetry to my TX.
/*
  crsf_sensor_vario_t crsfVario = { 0 };

  // Values are MSB first (BigEndian)
  crsfVario.verticalspd = htobe16((int16_t)(verticalspd*100.0));
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_VARIO, &crsfVario, sizeof(crsfVario));
*/
}

void telemetrySendVario( float verticalspd)
{
  crsf_sensor_vario_t crsfVario = { 0 };

  // Values are MSB first (BigEndian)
  crsfVario.verticalspd = htobe16((int16_t)(verticalspd*100.0));
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_VARIO, &crsfVario, sizeof(crsfVario));
}


void floatToString( char* buffer, size_t bufferSize,float value) {
   int32_t intValue = (int32_t)(value * 100.0 + 0.5); // Scale to preserve two decimal places
   if (intValue > 999999) intValue = 999999; // Cap to max displayable value
   if (intValue < -99999) intValue = -99999; // Cap to min displayable value
   if(intValue < 0)     snprintf(buffer, bufferSize, "%06ld", (long int)intValue);
   else                 snprintf(buffer, bufferSize, "%6ld" , (long int)intValue);
    buffer[7] = '\0'; // Ensure null termination
    buffer[6] = buffer[5]; // Move last digit to position 6
    buffer[5] = buffer[4]; // Move second last digit to position 5
    buffer[4] = '.'; // Insert decimal point at position 4
    if(buffer[5] == ' ') buffer[5] = '0'; // Replace space with '0' if needed
    if(buffer[3] == ' ') buffer[3] = '0'; // Replace space with '0' if needed
}


void baroSerialDisplayTask(uint32_t millis_now){
  static uint32_t last_millis=0;
  if (millis_now - last_millis < 250) return;
  last_millis = millis_now;
  
    static char float_string_buffer[8];

    floatToString(float_string_buffer, sizeof(float_string_buffer), baroTemperature);
    snprintf(debug_str_buffer, sizeof(debug_str_buffer), "Temperature = %s*C\n\r", float_string_buffer);
//    writeSerialTXFifoBuffer((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
  serial2TX.write((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
   // snprintf(debug_str_buffer, sizeof(debug_str_buffer), "Pressure = %4dhPa\n\r", (int)baroPressure);
   // writeSerialTXFifoBuffer((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
    floatToString(float_string_buffer, sizeof(float_string_buffer),filt_alt_AGL);
    snprintf(debug_str_buffer, sizeof(debug_str_buffer), "Approx altitude ASL = %3dm ; Altitude AGL = %sm\n\r", (int)filt_alt_ASL,float_string_buffer); 
    //writeSerialTXFifoBuffer((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
  serial2TX.write((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
    floatToString(float_string_buffer, sizeof(float_string_buffer), filt_vario);
    snprintf(debug_str_buffer, sizeof(debug_str_buffer), "Vario = %sm/s\n\r", float_string_buffer);
//    writeSerialTXFifoBuffer((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
  serial2TX.write((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
    //writeSerialTXFifoBuffer((uint8_t*)"\n\r", 2);
  serial2TX.write((uint8_t*)"\n\r", 2);


  
}

void baroProcessingTask(uint32_t millis_now){

  static uint32_t last_millis=0,loop_counter=0;
  if (millis_now - last_millis < 125) return;
  last_millis = millis_now;
  static uint32_t millis_start=0, millis_end=0;


  millis_start=HAL_GetTick();

  baroAltitude = BaroSensor.readPressureAltitudeMeter(1013.25);
  baroTemperature = BaroSensor.readTemperature();
  baroPressure = BaroSensor.readPressure();

  if (loop_counter <1000) {    // initialisation --> average ground altitude
    GND_altitude += baroAltitude;
    GND_alt_count++;
    filt_alt_ASL = baroAltitude;
  } else {
    filt_alt_ASL = (IIR_ALPHA * baroAltitude) + (IIR_BETA * filt_alt_ASL);
  }
  filt_alt_AGL =filt_alt_ASL - (GND_altitude / GND_alt_count);
  vario=(filt_alt_AGL-previous_alt_ASL)*20;
  filt_vario = (IIR_ALPHA * vario) + (IIR_BETA * filt_vario);
  previous_alt_ASL = filt_alt_AGL;
  loop_counter++;
  millis_end=HAL_GetTick();
  volatile uint32_t baro_processing_millis = millis_end - millis_start; // For debugging - check how long baro processing takes and if it causes delays in PWM output or CRSF reception
  snprintf(debug_str_buffer, sizeof(debug_str_buffer), "Baro processing time: %lu ms\n\r", (unsigned long)baro_processing_millis);
//  writeSerialTXFifoBuffer((uint8_t*)debug_str_buffer, strlen(debug_str_buffer));
  serial2TX.write((uint8_t*)debug_str_buffer, strlen(debug_str_buffer)); // Send baro processing time to UART2 for debugging
}



}  // extern "C"
