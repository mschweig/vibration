/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include "ui.h"
#include <logging/log.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <drivers/gpio.h>

static struct k_work calibration_empty_data_work;
static struct k_work calibration_half_data_work;
static struct k_work calibration_full_data_work;
static struct k_work measurement_data_work;

LOG_MODULE_REGISTER(algorithm, CONFIG_UI_LOG_LEVEL);

/*GPIO device*/
static const struct device *gpio_dev;
#define VIBRATION_PIN 16

/*MPU6050 Device*/
static const struct device *mpu6050;

/*Buttons*/
#define BUTTON_NODE    		DT_NODELABEL(button2) //= Button 1 on DK WTF
#define BUTTON_GPIO_LABEL	DT_GPIO_LABEL(BUTTON_NODE, gpios)
#define BUTTON_GPIO_PIN		DT_GPIO_PIN(BUTTON_NODE, gpios)
#define BUTTON_GPIO_FLAGS	GPIO_INPUT | DT_GPIO_FLAGS(BUTTON_NODE, gpios)

/*GPIO Callback*/
static struct gpio_callback gpio_cb;
static bool start_measurement = false;
static float calibration_state = 0;

/*Number of Sensor samples*/
#define NFFT 512

/* BIN to FREQUENCY Formula
(SIGNAL_FREQUENCY/BIN_INDEX) = (SAMPLE_FREQUENCY/FFT_SIZE*2) --> Bin 159 = 110 Hz

SAMPLE_FREQUENCY = 720Hz 
SIGNAL_FREQUENCY = 110Hz
*/

/*Storage for timesignal TODO: Non-Global*/
float32_t signal_empty[NFFT];
float32_t signal_half[NFFT];
float32_t signal_full[NFFT];
float32_t measurement[NFFT];
float32_t empty, half, full;

/*Calculate FFT*/
static float32_t cmsisFFT(float state){ 

	float32_t frequencyOutput[NFFT];
	uint32_t testIndex = 0;
	float32_t maxValue = 0;
	float32_t mean = 0;
	
	// RFFT instance
    arm_rfft_fast_instance_f32 S; 
    arm_status status;

	if (state == 0){
		//detrend the empty signal
		arm_mean_f32(signal_empty,NFFT,&mean);
		arm_offset_f32(signal_empty,-mean,signal_empty,NFFT);

		status = arm_rfft_fast_init_f32(&S, NFFT);
		arm_rfft_fast_f32(&S,signal_empty, frequencyOutput,0);

		if (status != ARM_MATH_SUCCESS){
			LOG_ERR("CMSIS-DSP Calculation Failed");
		}

		//remove DC-offset
		frequencyOutput[0] = 0;

		//calculate maximum frequency
		arm_abs_f32(frequencyOutput, frequencyOutput, NFFT);
		arm_max_f32(frequencyOutput,NFFT, &maxValue, &testIndex);
		
		printf("\nEmpty: Max-Bin: %d, Amplitude %f\r\n",testIndex,maxValue);
	}
	if (state == 0.5){
		//detrend the empty signal
		arm_mean_f32(signal_half,NFFT,&mean);
		arm_offset_f32(signal_half,-mean,signal_half,NFFT);

		status = arm_rfft_fast_init_f32(&S, NFFT);
		arm_rfft_fast_f32(&S,signal_half, frequencyOutput,0);

		if (status != ARM_MATH_SUCCESS){
			LOG_ERR("CMSIS-DSP Calculation Failed");
		}

		//remove DC-offset
		frequencyOutput[0] = 0;

		//calculate maximum frequency
		arm_abs_f32(frequencyOutput, frequencyOutput, NFFT);
		arm_max_f32(frequencyOutput,NFFT, &maxValue, &testIndex);
		
		printf("\nHalf: Max-Bin: %d, Amplitude %f\r\n",testIndex,maxValue);
	}
	if (state == 1){
		//detrend the empty signal
		arm_mean_f32(signal_full,NFFT,&mean);
		arm_offset_f32(signal_full,-mean,signal_full,NFFT);

		status = arm_rfft_fast_init_f32(&S, NFFT);
		arm_rfft_fast_f32(&S,signal_full, frequencyOutput,0);

		if (status != ARM_MATH_SUCCESS){
			LOG_ERR("CMSIS-DSP Calculation Failed");
		}

		//remove DC-offset
		frequencyOutput[0] = 0;

		//calculate maximum frequency
		arm_abs_f32(frequencyOutput, frequencyOutput, NFFT);
		arm_max_f32(frequencyOutput,NFFT, &maxValue, &testIndex);
		
		printf("\nFull: Max-Bin: %d, Amplitude %f\r\n",testIndex,maxValue);
	}
	if (state == 2){
		//detrend the empty signal
		arm_mean_f32(measurement,NFFT,&mean);
		arm_offset_f32(measurement,-mean,measurement,NFFT);

		status = arm_rfft_fast_init_f32(&S, NFFT);
		arm_rfft_fast_f32(&S,measurement, frequencyOutput,0);

		if (status != ARM_MATH_SUCCESS){
			LOG_ERR("CMSIS-DSP Calculation Failed");
		}

		//remove DC-offset
		frequencyOutput[0] = 0;

		//calculate maximum frequency
		arm_abs_f32(frequencyOutput, frequencyOutput, NFFT);
		arm_max_f32(frequencyOutput,NFFT, &maxValue, &testIndex);
		
		printf("\nGauge: Max-Bin: %d, Amplitude %f\r\n",testIndex,maxValue);
	}
	
	return maxValue;
}

/*Calculate frequency*/
static float calculateFrequency(int t_start, int t_end){
	int time = (t_end - t_start);   
	float frequency = (1/((float)time/NFFT)*1000);
	return frequency;
}

/*Control Vibration Motor*/
static void controlVibration(bool status){

	gpio_pin_set(gpio_dev, 16, status);

}

/*Get Sensor Signal*/
static float32_t processMPU6050()
{
	struct sensor_value temperature;
	struct sensor_value accel[3];

	int rc = sensor_sample_fetch(mpu6050);

	if (rc == 0) {
		rc = sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(mpu6050, SENSOR_CHAN_AMBIENT_TEMP,
					&temperature);
	}
	if (rc == 0) {
		/*
		printk("[%s]:%g Cel\n"
		       "  accel %f %f %f m/s/s\n"
		       "  gyro  %f %f %f rad/s\n",
		       now_str(),
		       sensor_value_to_double(&temperature),
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2])); */
		//printk("%f\n",sensor_value_to_double(&accel[2]));
	} else {
		LOG_ERR("sample fetch/get failed: %d", rc);
	}

	return sensor_value_to_double(&accel[2]);
}

/*Control Vibration Motor and Measure*/
static void measureVibration(){

	controlVibration(1);

	//wait until motor drives up
	k_sleep(K_SECONDS(2));
	
	if(calibration_state == 0){
		for (uint16_t i = 0; i<NFFT; i++){
			signal_empty[i] = processMPU6050();
			k_sleep(K_NSEC(600000)); //ca. 800 Hz TODO: trigger when new data is available
		}
	}
	
	if(calibration_state == 0.5){
		for (uint16_t i = 0; i<NFFT; i++){
			signal_half[i] = processMPU6050();
			k_sleep(K_NSEC(600000)); //ca. 800 Hz TODO: trigger when new data is available
		}
	}

	if(calibration_state == 1){
		for (uint16_t i = 0; i<NFFT; i++){
			signal_full[i] = processMPU6050();
			k_sleep(K_NSEC(600000)); //ca. 800 Hz TODO: trigger when new data is available
		}
	}

	if(calibration_state == 2){
		for (uint16_t i = 0; i<NFFT; i++){
			measurement[i] = processMPU6050();
			k_sleep(K_NSEC(600000)); //ca. 800 Hz TODO: trigger when new data is available
		}
	}

	controlVibration(0);
}

/*Calibration Mechanism*/
static float32_t calibration_empty(struct k_work *work_q){

	LOG_WRN("Started Calibration at empty level");
	measureVibration();
	empty = cmsisFFT(calibration_state);
	LOG_WRN("Press Button for Calibration at half level");
	calibration_state = 0.5;
	return empty;
}

static float32_t calibration_half(struct k_work *work_q){

	measureVibration();
	half = cmsisFFT(calibration_state);
	LOG_WRN("Press Button for Calibration at full level");
	calibration_state = 1;
	return half;
}

static float32_t calibration_full(struct k_work *work_q){

	measureVibration();
	full = cmsisFFT(calibration_state);
	LOG_WRN("Calibration finished!");
	calibration_state = 2;
	return full;
}

static float32_t measureLevel(struct k_work *work_q){

	LOG_WRN("Started Non-Invasive Fill Level Measurement");
	measureVibration();
	float32_t level = cmsisFFT(calibration_state);

	float P1x = full; 
	float P1y = 100;
	float P2x = half;
	float P2y = 50;
	float P3x = empty; 
	float P3y = 0;
	float k1, d1, k2, d2, y;

	if (level < P2x){
		k1 = (P2y-P1y) / (P2x-P1x);
		d1 = (P2y - (P2x*k1));
		y = k1*(level)+d1;
		printk("Measured Level: %f \n", y);
	}
	if (level > P2x){
		k2 = (P3y-P2y) / (P3x-P2x);
		d2 = (P3y - (P3x*k2));
		y = k2*(level)+d2;
		printk("Measured Level: %f \n", y);
	}

	if (level < full){
		printk("Measured Level: 100 \n");
	}

	if (level > empty){
		printk("Measured Level: 0 \n");
	}
	
	return y;
}

void button_pressed_callback(const struct device *gpiob, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	LOG_DBG("Button pressed");

	start_measurement = true;

	if (calibration_state == 0){
		//call calibration in workqueue
		k_work_submit(&calibration_empty_data_work);
	}
	if (calibration_state == 0.5){
		//call calibration in workqueue
		k_work_submit(&calibration_half_data_work);
	}
	if (calibration_state == 1){
		//call calibration in workqueue
		k_work_submit(&calibration_full_data_work);
	}
	if (calibration_state == 2){
		//call measurement in workqueue
		k_work_submit(&measurement_data_work);
	}

	
}

bool init_button(void)
{
	int ret = gpio_pin_configure(gpio_dev, BUTTON_GPIO_PIN, BUTTON_GPIO_FLAGS);
	if (ret != 0) {
        LOG_ERR("Error %d: failed to configure %s pin %d\n",
            	ret, BUTTON_GPIO_LABEL, BUTTON_GPIO_PIN);
        
		return false;
    }

    ret = gpio_pin_interrupt_configure(gpio_dev,
                                       BUTTON_GPIO_PIN,
                                       GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("Error %d: failed to configure interrupt on %s pin %d\n",
               ret, BUTTON_GPIO_LABEL, BUTTON_GPIO_PIN);

        return false;
    }

    gpio_init_callback(&gpio_cb, button_pressed_callback, BIT(BUTTON_GPIO_PIN));
    gpio_add_callback(gpio_dev, &gpio_cb);

	return true;
}

void algorithm_init(struct k_work_q *work_q)
{
	/*Initialize GPIO Pin*/
	gpio_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	if (!gpio_dev){
           LOG_ERR("Error binding GPIO-PORT 0");
           return 0;
	}
	gpio_pin_configure(gpio_dev, VIBRATION_PIN, GPIO_OUTPUT);

	/*initialize Button*/
	init_button();

	/*Initialize MPU6050*/
	mpu6050 = device_get_binding(DT_LABEL(DT_INST(0, invensense_mpu6050)));
	if (!mpu6050) {
		LOG_ERR("Failed to find sensor MPU6050");
		return;
	}
	processMPU6050();

	/*Init workqueue Items for Calibration and algorithm*/
	k_work_init(&calibration_empty_data_work, calibration_empty);
	k_work_init(&calibration_half_data_work, calibration_half);
	k_work_init(&calibration_full_data_work, calibration_full);
	k_work_init(&measurement_data_work, measureLevel);
	

	LOG_WRN("Press Button to start Calibration at empty level");
	
}