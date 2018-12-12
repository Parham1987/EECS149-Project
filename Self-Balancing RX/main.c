// Self-Balancing Robot RX
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "nrf.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrfx_saadc.h"
#include "nrf_twi_mngr.h"
#include "nrf_drv_spi.h"

#include "app_error.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "buckler.h"
#include "display.h"
#include "gpio.h"
#include "mpu9250.h"
#include "simple_ble.h"

// PID parameters
double kp=15, ki=1.5, kd=6;
float theta,speed,setpoint=90,lastInput=90,ITerm=0,error,dInput;

// BLE readings 
uint8_t x_val,y_val;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// GPI parameters
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

// output and input pins
#define X_CHANNEL 0
#define Y_CHANNEL 1
#define Z_CHANNEL 2
#define A0 4  //channel 0
#define A1 3  //channel 1
#define D0 2  //channel 2
#define D1 5  //channel 3
#define switch0 22 //On/Off switch

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

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

nrf_pwm_values_individual_t seq_values[] = {0, 0, 0, 0};
nrf_pwm_sequence_t const seq =
{
    .values.p_individual = seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats         = 0,
    .end_delay       = 0
};


// Set duty cycle between 0 and 100%
void pwm_update_duty_cycle_ch0(uint8_t duty_cycle)
{
    // Check if value is outside of range. If so, set to 100%
    if(duty_cycle >= 100)
    {
        seq_values->channel_0 = 100;
    }
    else
    {
        seq_values->channel_0 = duty_cycle;
    }
    if(duty_cycle <= 0 )
    {
    	seq_values->channel_0 = 0;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}
void pwm_update_duty_cycle_ch1(uint8_t duty_cycle)
{
    // Check if value is outside of range. If so, set to 100%
    if(duty_cycle >= 100)
    {
        seq_values->channel_1 = 100;
    }
    else
    {
        seq_values->channel_1 = duty_cycle;
    }
    if(duty_cycle <= 0 )
    {
      seq_values->channel_1 = 0;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}
void pwm_update_duty_cycle_ch2(uint8_t duty_cycle)
{
    // Check if value is outside of range. If so, set to 100%
    if(duty_cycle >= 100)
    {
        seq_values->channel_2 = 100;
    }
    else
    {
        seq_values->channel_2 = duty_cycle;
    }
    if(duty_cycle <= 0 )
    {
      seq_values->channel_2 = 0;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}
void pwm_update_duty_cycle_ch3(uint8_t duty_cycle)
{
    // Check if value is outside of range. If so, set to 100%
    if(duty_cycle >= 100)
    {
        seq_values->channel_3 = 100;
    }
    else
    {
        seq_values->channel_3 = duty_cycle;
    }
    if(duty_cycle <= 0 )
    {
      seq_values->channel_3 = 0;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

static void pwm_init(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            A0,             // channel 0
            A1,             // channel 1
            D0,             // channel 2
            D1,             // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 100,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    // Init PWM without error handler
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
    
}

// BLE configuration
// This is mostly irrelevant since we are scanning only
static simple_ble_config_t ble_config = {
        // BLE address is c0:98:e5:49:00:02
        .platform_id       = 0x49,    // used as 4th octet in device BLE address
        .device_id         = 0x0002,  // used as the 5th and 6th octet in the device BLE address, you will need to change this for each device you have
        .adv_name          = "EE149", // irrelevant in this example
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS), // send a packet once per second (minimum is 20 ms)
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS), // irrelevant if advertising only
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS), // irrelevant if advertising only
};
simple_ble_app_t* simple_ble_app;

void ble_evt_adv_report(ble_evt_t const* p_ble_evt) {

  // extract the fields we care about
  ble_gap_evt_adv_report_t const* adv_report = &(p_ble_evt->evt.gap_evt.params.adv_report);
  uint8_t const* ble_addr = adv_report->peer_addr.addr;
  uint8_t* adv_buf = adv_report->data.p_data;
  uint16_t adv_len = adv_report->data.len;

  // print the BLE address of the device
  //printf("BLE Address: %X%X%X%X%X%X\n", ble_addr[5], ble_addr[4], ble_addr[3], ble_addr[2], ble_addr[1], ble_addr[0]);
  if (ble_addr[5] == 0xC0 && ble_addr[4] == 0x98 && ble_addr[3] == 0xE5 && ble_addr[2] == 0x49) {
    if (adv_len==2)
    {
      x_val=adv_buf[8];
      y_val=adv_buf[9];
    }
  }
}

//Movement functions
void backward(float speed)
{
  pwm_update_duty_cycle_ch0(0);
  pwm_update_duty_cycle_ch1(speed);
  pwm_update_duty_cycle_ch2(speed);
  pwm_update_duty_cycle_ch3(0);
}
void forward(float speed)
{
  pwm_update_duty_cycle_ch0(speed);
  pwm_update_duty_cycle_ch1(0);
  pwm_update_duty_cycle_ch2(0);
  pwm_update_duty_cycle_ch3(speed);
}
void turn_left(float speed)
{
  pwm_update_duty_cycle_ch0(0);
  pwm_update_duty_cycle_ch1(speed);
  pwm_update_duty_cycle_ch2(0);
  pwm_update_duty_cycle_ch3(speed);
}
void turn_right(float speed)
{
  pwm_update_duty_cycle_ch0(speed);
  pwm_update_duty_cycle_ch1(0);
  pwm_update_duty_cycle_ch2(speed);
  pwm_update_duty_cycle_ch3(0);
}
void forward_right(float speed)
{
  pwm_update_duty_cycle_ch0(speed);
  pwm_update_duty_cycle_ch1(0);
  pwm_update_duty_cycle_ch2(0);
  pwm_update_duty_cycle_ch3(0);
}
void backward_right(float speed)
{
  pwm_update_duty_cycle_ch0(0);
  pwm_update_duty_cycle_ch1(speed);
  pwm_update_duty_cycle_ch2(0);
  pwm_update_duty_cycle_ch3(0);
}
void backward_left(float speed)
{
  pwm_update_duty_cycle_ch0(0);
  pwm_update_duty_cycle_ch1(0);
  pwm_update_duty_cycle_ch2(speed);
  pwm_update_duty_cycle_ch3(0);
}
void forward_left(float speed)
{
  pwm_update_duty_cycle_ch0(0);
  pwm_update_duty_cycle_ch1(0);
  pwm_update_duty_cycle_ch2(0);
  pwm_update_duty_cycle_ch3(speed);
}
void stop()
{
  pwm_update_duty_cycle_ch0(0);
  pwm_update_duty_cycle_ch1(0);
  pwm_update_duty_cycle_ch2(0);
  pwm_update_duty_cycle_ch3(0);
}

// our PID implementation
bool PID_Compute() {
    error = setpoint - theta;
    ITerm += (ki * error);
    if (ITerm > 100)
      ITerm = 100;
    else if (ITerm < -100)
      ITerm = -100;
    dInput = (theta - lastInput);

    /*Compute PID Output*/
    speed = kp * error + ITerm - kd * dInput;

    if (speed > 100)
      speed = 100;
    else if (speed < -100)
      speed = -100;
    /*Remember some variables for next time*/
    lastInput = theta;
    return true;
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);

  name1 * first = (name1 *) 0x50000504;
  name2 * second = (name2 *) 0x50000700;

  second->PIN_CNF_22 = 0;
  second->PIN_CNF_28 = 0;
  second->PIN_CNF_4 = 0;
  second->PIN_CNF_3 = 0;
  int dir=1;
  gpio_config(A0,dir);
  gpio_config(A1,dir);
  gpio_config(D0,dir);
  gpio_config(D1,dir);

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
  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);


  pwm_init();

  // Setup BLE
  // Note: simple BLE is our own library. You can find it in `nrf5x-base/lib/simple_ble/`
  simple_ble_app = simple_ble_init(&ble_config);
  advertising_stop();

  // Start scanning
  scanning_start();
  bool sw0;
  int m_trig_high=150;
  int m_trig_low=110;
  char buf[16];

  // initialize MPU-9250 driver
  mpu9250_init(&twi_mngr_instance);
  printf("MPU-9250 initialized\n");

  int counter=0;
  // go into low power mode
  while(1) {
  	counter++;
    mpu9250_measurement_t acc_measurement = mpu9250_read_accelerometer();
    
    float x_val_acc = acc_measurement.x_axis;
    float y_val_acc = acc_measurement.y_axis;
    float z_val_acc = acc_measurement.z_axis;

    theta = atan((y_val_acc)/(z_val_acc))*360/3.1415+90;
    PID_Compute();
    // display results
    if (counter==100){
		printf("                      X-Axis\t    Y-Axis\t    Z-Axis\n");
    	printf("                  ----------\t----------\t----------\n");
    	printf("Acceleration (g): %10.3f\t%10.3f\t%10.3f\n", acc_measurement.x_axis, acc_measurement.y_axis, acc_measurement.z_axis);

    	printf("x: %f\ty: %f\tz:%f\ntheta:%f\n", x_val_acc, y_val_acc, z_val_acc, theta)*360/3.1415;
		printf("error %f , Iterm %f dInput %f speed %f last %f\n",error,ITerm,dInput,speed,lastInput );

        printf("output is %f\n",speed);
        counter=0;
    }
    sw0=gpio_read(switch0);
    if (sw0){
      if (theta<50 || theta>130)
      {
      	printf("Balancing\n");
      	if (speed>0)
      	{
      		backward(speed);
      	}
      	else 
      	{
      		forward(speed*-1);
      	}
      }
      else if (x_val>m_trig_high && y_val<m_trig_high && y_val>m_trig_low)
      {       
      	printf("forward\n");
        forward(60);
        nrf_delay_ms(100);
      }
      else if (x_val<m_trig_low && y_val<m_trig_high && y_val>m_trig_low)
      {
      	printf("backward\n");
        backward(60);
        nrf_delay_ms(100);
      }
      else if (x_val>m_trig_low && x_val<m_trig_high && y_val>m_trig_high)
      {
      	printf("right turn\n");
		turn_right(80);
      }
      else if (x_val>m_trig_low && x_val<m_trig_high && y_val<m_trig_low)
      {
      	printf("left trun\n");
      	turn_left(80);
      }
      else if (x_val>m_trig_high && y_val>m_trig_high)
      {
      	printf("forward right\n");
        forward_right(80);
      }
      else if (x_val>m_trig_high && y_val<m_trig_low)
      {
        printf("forward left\n"); 
        forward_left(80);
      }
      else if (x_val<m_trig_low && y_val>m_trig_high)
      {
      	printf("backward right\n");
        backward_right(80);
      }
      else if (x_val<m_trig_low && y_val<m_trig_low)
      {
      	printf("backward left\n");
        backward_left(80);
      }
      else
      {
		stop();       
      }
    }
    else
    {
    	stop();
    }
    power_manage();
  }
}



