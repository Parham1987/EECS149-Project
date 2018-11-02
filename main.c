// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"


#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"

#include "buckler.h"
#include "gpio.h"
// ADC channels
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2

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

typedef enum {
  FORWARD,
  BACKWARD,
  BALANCE,
} CarState_t;

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;
  //uint32_t * GPIO_OUT = 0x50000504;
  //uint32_t * GPIO_DIR = 0x50000514;
  //uint32_t * GPIO_LED = 0x5000075C;

  name1 * first = (name1 *) 0x50000504;
  name2 * second = (name2 *) 0x50000700;
  

  second->PIN_CNF_22 = 0;
  second->PIN_CNF_28 = 0;

  
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
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_X;
  error_code = nrfx_saadc_channel_init(X_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Y;
  error_code = nrfx_saadc_channel_init(Y_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);

  // specify input pin and initialize that ADC channel
  channel_config.pin_p = BUCKLER_ANALOG_ACCEL_Z;
  error_code = nrfx_saadc_channel_init(Z_CHANNEL, &channel_config);
  APP_ERROR_CHECK(error_code);


  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // loop forever
  //*GPIO_DIR = 1 << 24;
  
 gpio_direction_t dir = 1;
 gpio_config(23, dir);
 gpio_config(2, dir);
 gpio_config(24, dir);
 gpio_config(25, dir);

 float ADC_z_max = 2090;
 float ADC_z_min = 1220;
 float ADC_X_max = 2033;
 float ADC_X_min = 1160;
 float a_z = (ADC_z_max+ADC_z_min)/2;
 float a_X = (ADC_X_max+ADC_X_min)/2;
 CarState_t state = FORWARD;

 while (1) {
  	
  	uint32_t switch0 = gpio_read(22);
  	uint32_t button0 = gpio_read(28);
    nrf_saadc_value_t x_val = sample_value(X_CHANNEL);
    nrf_saadc_value_t y_val = sample_value(Y_CHANNEL);
    nrf_saadc_value_t z_val = sample_value(Z_CHANNEL);

    float theta = atan((z_val-a_z)/(x_val-a_X))*180/3.1415;

    // display results
    printf("x: %d\ty: %d\tz:%d\ntheta:%f\n", x_val, y_val, z_val, theta);
    if (!switch0)
    {
    	switch (state)
    	{
    		case BALANCE:
    		{
    		    gpio_clear(25);
  				gpio_set(2);
  				gpio_set(24);
  				nrf_delay_ms(80);
  				gpio_clear(24);
  				gpio_clear(25);
  				gpio_clear(2);
  				nrf_delay_ms(80);
  				gpio_clear(24);
  				gpio_clear(2);
  				gpio_set(25);

  				nrf_delay_ms(80);
  				printf("BALANCE\n");			
  				if (abs(theta)<75 && theta >0)
  				{
  					state=BACKWARD;
  				}
  				if (abs(theta)<75 && theta <0)
  				{
  					state=FORWARD;
  				}
  				break;
  			}
    		case FORWARD:
    		{
    			gpio_clear(24);
  				gpio_clear(25);
  				gpio_clear(2);
  				nrf_delay_ms(400);
  				gpio_set(2);
  				gpio_set(24);
  				nrf_delay_ms(800);
  				if (abs(theta)>75)
  				{
	  				state=BALANCE;
  				}
  				printf("FORWARD\n");
	  			break;
  			}
  			case BACKWARD:
  			    gpio_clear(24);
  				gpio_clear(25);
  				gpio_clear(2);
  				nrf_delay_ms(400);
  				gpio_set(25);
  				nrf_delay_ms(800);
  				if (abs(theta)>75)
  				{
  					state=BALANCE;
  				}
  				printf("BACKWARD\n");
  				break;
    	}
	}
   	else{
  		gpio_clear(24);
  		gpio_clear(25);
  		gpio_clear(2);
  	}
  	


  	nrf_delay_ms(100);
  }
}