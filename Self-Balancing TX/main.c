// Self-Balancing Robot TX
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_saadc.h"
#include "nrf_serial.h"

#include "app_error.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "buckler.h"
#include "gpio.h"
#include "simple_ble.h"

// Joystick readings
nrf_saadc_value_t x_val,y_val;

typedef struct{
  uint32_t GPIO_OUT;
  uint32_t GPIO_OUTSET;
  uint32_t GPIO_OUTCLR;
  uint32_t GPIO_IN;
  uint32_t GPIO_DIR;
  uint32_t GPIO_DIRSET;
  uint32_t GPIO_DIRCLR;
  uint32_t GPIO_LATCH;
  uint32_t GPIO_DETECTMODE;

}name1;
typedef struct{
  uint32_t PIN_CNF_0;
  uint32_t PIN_CNF_1;
  uint32_t PIN_CNF_2;
  uint32_t PIN_CNF_3;
  uint32_t PIN_CNF_4;
  uint32_t PIN_CNF_5;
  uint32_t PIN_CNF_6;
  uint32_t PIN_CNF_7;
  uint32_t PIN_CNF_8;
  uint32_t PIN_CNF_9;
  uint32_t PIN_CNF_10;
  uint32_t PIN_CNF_11;
  uint32_t PIN_CNF_12;
  uint32_t PIN_CNF_13;
  uint32_t PIN_CNF_14;
  uint32_t PIN_CNF_15;
  uint32_t PIN_CNF_16;
  uint32_t PIN_CNF_17;
  uint32_t PIN_CNF_18;
  uint32_t PIN_CNF_19;
  uint32_t PIN_CNF_20;
  uint32_t PIN_CNF_21;
  uint32_t PIN_CNF_22;
  uint32_t PIN_CNF_23;
  uint32_t PIN_CNF_24;
  uint32_t PIN_CNF_25;
  uint32_t PIN_CNF_26;
  uint32_t PIN_CNF_27;
  uint32_t PIN_CNF_28;
  uint32_t PIN_CNF_29;
  uint32_t PIN_CNF_30;
  uint32_t PIN_CNF_31;
}name2;

#define X_CHANNEL 0
#define Y_CHANNEL 1

#define BUCKLER_ANALOG_X NRF_SAADC_INPUT_AIN1 // P0.03
#define BUCKLER_ANALOG_Y NRF_SAADC_INPUT_AIN2 // P0.04

// callback for SAADC events
void saadc_callback (nrfx_saadc_evt_t const * p_event) {
  // don't care about adc callbacks
}

// sample a particular analog channel in blocking mode
nrf_saadc_value_t sample_value (uint8_t channel) {
  nrf_saadc_value_t val;
  ret_code_t error_code = nrfx_saadc_sample_convert(channel, &val);
  APP_ERROR_CHECK(error_code);
  return val;
}

// Create a timer
APP_TIMER_DEF(adv_timer);

// BLE configuration
static simple_ble_config_t ble_config = {
        // BLE address is c0:98:e5:49:00:01
        .platform_id       = 0x49,    // used as 4th octet in device BLE address
        .device_id         = 0x0001,  // used as the 5th and 6th octet in the device BLE address, you will need to change this for each device you have
        .adv_name          = "EE149", // irrelevant in this example
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS), // send a packet once per second (minimum is 20 ms)
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS), // irrelevant if advertising only
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS), // irrelevant if advertising only
};
simple_ble_app_t* simple_ble_app;


// Sends the specified data over BLE advertisements
void set_ble_payload(uint8_t* buffer, uint8_t length) {
  static uint8_t adv_buffer[24] = {0};
  static ble_advdata_manuf_data_t adv_payload = {
    .company_identifier = 0x02E0, // Lab11 company ID (University of Michigan)
    .data.p_data = adv_buffer,
    .data.size = 24,
  };

  // copy over up to 23 bytes of advertisement payload
  adv_buffer[0] = 0x02; // identifies a Buckler advertisement payload
  if (length > 02) {
    length = 2; // maximum size is 23 bytes of payload
  }
  memcpy(&(adv_buffer[1]), buffer, length);
  adv_payload.data.size = 1+length;

  // create an advertisement with a manufacturer-specific data payload
  // https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.0.0%2Fstructble__advdata__t.html
  ble_advdata_t advdata = {0};
  advdata.name_type = BLE_ADVDATA_NO_NAME; // do not include device name (adv_name) in advertisement
  advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; // BLE Low energy advertisement
  advdata.p_manuf_specific_data = &adv_payload;

  // update advertisement data and start advertising
  simple_ble_set_adv(&advdata, NULL);
}

// Callback when the timer fires. Updates the advertisement data
void adv_timer_callback(void) {
  static uint8_t increment = 0;

  // Update advertisement data
  // Increments each value by one each time
  increment++;
  uint8_t buffer[2] = {0};
  // set BLE buffers to joystick's readings
  buffer[0]=x_val;
  buffer[1]=y_val;
  printf("Buffer value: %d\n", buffer[0]);
  set_ble_payload(buffer, 2);
}


int main(void) {
  name1 * first = (name1 *) 0x50000504;
  name2 * second = (name2 *) 0x50000700;
  
  second->PIN_CNF_4 = 0;
  second->PIN_CNF_3 = 0;

  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  // initialize analog to digital converter
  nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;
  error_code = nrfx_saadc_init(&saadc_config, saadc_callback);
  APP_ERROR_CHECK(error_code);

  // initialize analog inputs
  // configure with 0 as input pin for now
  nrf_saadc_channel_config_t channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(0);
  channel_config.gain = NRF_SAADC_GAIN1_6; // input gain of 1/6 Volts/Volt, multiply incoming signal by (1/6)
  channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // 0.6 Volt reference, input after gain can be 0 to 0.6 Volts

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_X;
  error_code = nrfx_saadc_channel_init(X_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_Y;
  error_code = nrfx_saadc_channel_init(Y_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // Setup BLE
  // Note: simple BLE is our own library. You can find it in `nrf5x-base/lib/simple_ble/`
  simple_ble_app = simple_ble_init(&ble_config);

  // Set advertisement data. Maximum size is 23 bytes of payload
  uint8_t buffer[2] = {0};
  for (int i=0; i<2; i++) {
    buffer[i] = i;
  }
  set_ble_payload(buffer, 2);

  // Set a timer to change the data. Data could also be changed after sensors
  // are read in real applications
  app_timer_init();
  app_timer_create(&adv_timer, APP_TIMER_MODE_REPEATED, (app_timer_timeout_handler_t)adv_timer_callback);
  app_timer_start(adv_timer, APP_TIMER_TICKS(20), NULL); // 1000 milliseconds

  // go into low power mode
  while(1) {
    power_manage();
    x_val = abs(sample_value(X_CHANNEL)*255/3300);
    y_val = abs(sample_value(Y_CHANNEL)*255/3300);
    // display results
    printf("x: %d\ty: %d\t\n", x_val, y_val);
    nrf_delay_ms(1);
  }
}

