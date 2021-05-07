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

LOG_MODULE_REGISTER(algorithm, CONFIG_UI_LOG_LEVEL);

/*GPIO device*/
static const struct device *gpio_dev;
#define VIBRATION_PIN 16

/*Buttons*/
#define BUTTON_NODE    		DT_NODELABEL(button2) //= Button 1 on DK WTF
#define BUTTON_GPIO_LABEL	DT_GPIO_LABEL(BUTTON_NODE, gpios)
#define BUTTON_GPIO_PIN		DT_GPIO_PIN(BUTTON_NODE, gpios)
#define BUTTON_GPIO_FLAGS	GPIO_INPUT | DT_GPIO_FLAGS(BUTTON_NODE, gpios)
/*GPIO Callback*/
static struct gpio_callback gpio_cb;

static bool button_pressed = false;
static bool start_calibration = false;

/*Number of Sensor samples*/
#define NFFT 512

/* BIN to FREQUENCY Formula
(SIGNAL_FREQUENCY/BIN_INDEX) = (SAMPLE_FREQUENCY/FFT_SIZE*2) --> Bin 159 = 110 Hz
*/

/*Calculate FFT*/
static float32_t cmsisFFT(){ 

	/*Vibration Signal*/
	float32_t timeInput[NFFT] =
	{
	-1.527501,
	-0.378284,
	-0.169989,
	-1.122881,
	-1.738191,
	-2.212243,
	-1.936910,
	-0.531513,
	-0.184354,
	-0.605734,
	-1.285687,
	-2.188301,
	-2.229002,
	-0.852336,
	-0.306458,
	-0.337583,
	-1.022325,
	-2.087744,
	-2.252944,
	-1.201890,
	-0.526725,
	-0.189142,
	-0.634464,
	-1.893814,
	-2.178724,
	-1.611298,
	-0.873884,
	-0.148441,
	-0.287305,
	-1.498771,
	-1.977611,
	-2.001553,
	-1.307235,
	-0.318429,
	-0.217873,
	-0.541090,
	-1.692701,
	-2.176330,
	-1.714249,
	-0.893038,
	-0.143653,
	-0.232238,
	-1.489194,
	-2.037466,
	-2.102110,
	-1.295264,
	-0.313641,
	-0.136470,
	-1.146823,
	-1.764527,
	-2.269704,
	-1.915362,
	-0.454899,
	-0.179566,
	-0.699107,
	-1.405397,
	-2.183512,
	-2.123657,
	-0.758962,
	-0.282516,
	-0.380679,
	-1.055843,
	-2.116475,
	-2.183512,
	-1.197101,
	-0.600945,
	-0.246603,
	-0.725444,
	-1.927333,
	-2.207454,
	-1.580174,
	-0.876278,
	-0.057461,
	-0.323218,
	-0.974441,
	-2.020707,
	-2.224214,
	-1.223438,
	-0.588974,
	-0.110134,
	-0.598551,
	-1.886631,
	-2.142811,
	-1.640029,
	-0.833183,
	-0.064644,
	-0.232238,
	-1.474829,
	-2.035072,
	-2.003947,
	-1.259351,
	-0.239421,
	-0.201113,
	-1.352724,
	-1.867478,
	-2.205060,
	-1.719037,
	-0.402226,
	-0.100557,
	-0.928951,
	-1.568203,
	-2.221820,
	-2.087744,
	-0.632070,
	-0.210690,
	-0.519542,
	-1.189919,
	-2.123657,
	-2.193089,
	-1.046267,
	-0.423774,
	-0.217873,
	-0.790087,
	-1.486800,
	-2.255338,
	-2.133234,
	-0.742203,
	-0.292093,
	-0.399832,
	-1.012748,
	-2.032678,
	-2.135628,
	-1.082180,
	-0.404621,
	-0.189142,
	-0.620099,
	-1.821988,
	-2.135628,
	-1.532290,
	-0.811635,
	-0.179566,
	-0.342371,
	-1.623269,
	-1.970428,
	-2.037466,
	-1.295264,
	-0.256180,
	-0.055067,
	-1.240197,
	-1.860295,
	-2.154782,
	-1.661576,
	-0.284910,
	-0.031125,
	-0.871490,
	-1.568203,
	-2.202666,
	-2.003947,
	-0.641647,
	-0.220267,
	-0.495600,
	-1.180342,
	-2.126052,
	-2.149994,
	-1.747768,
	-0.435745,
	-0.155624,
	-0.787693,
	-1.541866,
	-2.185907,
	-2.128446,
	-0.660800,
	-0.210690,
	-0.454899,
	-1.065420,
	-2.070985,
	-2.205060,
	-1.194707,
	-0.555455,
	-0.253786,
	-0.730232,
	-1.975217,
	-2.221820,
	-1.637634,
	-0.878673,
	-0.248997,
	-0.380679,
	-1.616087,
	-1.980005,
	-2.068591,
	-1.280898,
	-0.217873,
	-0.201113,
	-1.278504,
	-1.910573,
	-2.205060,
	-1.807623,
	-0.399832,
	-0.189142,
	-0.912191,
	-1.570597,
	-2.207454,
	-2.116475,
	-0.579397,
	-0.184354,
	-0.407015,
	-1.072603,
	-2.169147,
	-2.229002,
	-1.086968,
	-0.435745,
	-0.280122,
	-0.646435,
	-1.364695,
	-2.202666,
	-2.097321,
	-0.964864,
	-0.342371,
	-0.363919,
	-0.926557,
	-2.121263,
	-2.178724,
	-1.137246,
	-0.545878,
	-0.311247,
	-0.665589,
	-1.905785,
	-2.154782,
	-1.740585,
	-0.909797,
	-0.160412,
	-0.263363,
	-1.647211,
	-2.047043,
	-1.946486,
	-1.367090,
	-0.296882,
	-0.217873,
	-1.158794,
	-1.735797,
	-2.128446,
	-1.731008,
	-0.521936,
	-0.237026,
	-0.675165,
	-1.414974,
	-2.109292,
	-2.039860,
	-0.814029,
	-0.342371,
	-0.272939,
	-0.883461,
	-2.039860,
	-2.145205,
	-1.965640,
	-0.663194,
	-0.208296,
	-0.560244,
	-1.242591,
	-2.078168,
	-2.133234,
	-1.048661,
	-0.395044,
	-0.292093,
	-0.842759,
	-2.008736,
	-2.109292,
	-1.460464,
	-0.766145,
	-0.203508,
	-0.450110,
	-1.699884,
	-2.085350,
	-1.877055,
	-1.237803,
	-0.296882,
	-0.246603,
	-1.450887,
	-1.927333,
	-2.075773,
	-1.683124,
	-0.407015,
	-0.232238,
	-0.984017,
	-1.654394,
	-2.094927,
	-1.982399,
	-0.605734,
	-0.232238,
	-0.517148,
	-1.211467,
	-2.080562,
	-2.126052,
	-1.005565,
	-0.450110,
	-0.213084,
	-0.873884,
	-1.496377,
	-2.128446,
	-2.130840,
	-0.742203,
	-0.237026,
	-0.402226,
	-1.134852,
	-2.094927,
	-2.214637,
	-1.115698,
	-0.483629,
	-0.213084,
	-0.720655,
	-1.872266,
	-2.188301,
	-1.534684,
	-0.876278,
	-0.124499,
	-0.351948,
	-1.640029,
	-2.054226,
	-1.881843,
	-1.266533,
	-0.208296,
	-0.186748,
	-1.252168,
	-1.771710,
	-2.135628,
	-1.604116,
	-0.502783,
	-0.158018,
	-0.799664,
	-1.505953,
	-2.212243,
	-1.980005,
	-0.701502,
	-0.186748,
	-0.387861,
	-1.139640,
	-2.197878,
	-2.188301,
	-1.055843,
	-0.517148,
	-0.076615,
	-0.873884,
	-1.460464,
	-2.226608,
	-2.135628,
	-0.758962,
	-0.284910,
	-0.445322,
	-1.077391,
	-2.030284,
	-2.209849,
	-1.149217,
	-0.591368,
	-0.134076,
	-0.656012,
	-1.874660,
	-2.224214,
	-1.587356,
	-0.890644,
	-0.062250,
	-0.270545,
	-1.551443,
	-2.070985,
	-2.054226,
	-1.288081,
	-0.136470,
	-0.117316,
	-1.302446,
	-1.841142,
	-2.178724,
	-1.625663,
	-0.476447,
	-0.107740,
	-0.837971,
	-1.513136,
	-2.205060,
	-2.015918,
	-0.749386,
	-0.234632,
	-0.553061,
	-1.132458,
	-1.750162,
	-2.260127,
	-1.721431,
	-0.392650,
	-0.071827,
	-0.758962,
	-1.441310,
	-2.260127,
	-2.099715,
	-0.739809,
	-0.301670,
	-0.474052,
	-1.110910,
	-2.037466,
	-2.173936,
	-0.962470,
	-0.445322,
	-0.270545,
	-0.782904,
	-1.968034,
	-2.205060,
	-1.558626,
	-0.873884,
	-0.165200,
	-0.387861,
	-1.620875,
	-2.104504,
	-1.908179,
	-1.118093,
	-0.079009,
	-0.119711,
	-1.304840,
	-1.833959,
	-2.161965,
	-1.618481,
	-0.296882,
	-0.055067,
	-0.916980,
	-1.541866,
	-2.236185,
	-2.003947,
	-0.620099,
	-0.160412,
	-0.126893,
	-1.254562,
	-1.745374,
	-2.178724,
	-1.644817,
	-0.385467,
	-0.076615,
	-0.967258,
	-1.505953,
	-2.272098,
	-2.037466,
	-0.543484,
	-0.186748,
	-0.517148,
	-1.221043,
	-2.181118,
	-2.173936,
	-1.019930,
	-0.385467,
	-0.217873,
	-0.723049,
	-1.936910,
	-2.248156,
	-1.312023,
	-0.694319,
	-0.093374,
	-0.462081,
	-1.754950,
	-2.114081,
	-1.766921,
	-1.012748,
	-0.069432,
	-0.146047,
	-1.285687,
	-1.819594,
	-1.999159,
	-1.347936,
	-0.158018,
	-0.033519,
	-0.960075,
	-1.640029,
	-2.142811,
	-1.829171,
	-1.113304,
	-0.174777,
	-0.251392,
	-1.343148,
	-1.800440,
	-2.176330,
	-1.606510,
	-0.361525,
	0.031124,
	-0.763751,
	-1.493982,
	-2.221820,
	-1.884237,
	-0.651223,
	-0.062250,
	-0.502783,
	-1.063026,
	-2.147599,
	-2.310405,
	-0.893038,
	-0.742203,
	-0.304064,
	-0.656012,
	-1.991976,
	-2.250550,
	-1.400608,
	-0.538696,
	-0.055067,
	-0.169989,
	-1.599327,
	-2.317588,
	-1.982399,
	-1.640029,
	-1.000777,
	-1.482011,
	-1.225832,
	-3.047819,
	-3.356671,
	-1.817200,
	-0.984017,
	0.153228,
	0.016759,
	-0.816423,
	-1.098939,
	-0.814029,
	-0.232238,
	0.280121,
	0.009576,
	-0.845154,
	-2.102110,
	-2.521095,
	-1.625663,
	-0.847548,
	-0.086192,
	-0.418986,
	};
	float32_t frequencyOutput[NFFT];
	uint32_t testIndex;
	float32_t maxValue;
	float32_t mean;

	// RFFT instance
    arm_rfft_fast_instance_f32 S; 
    arm_status status;

	//detrend the signal
	arm_mean_f32(timeInput,NFFT,&mean);
	arm_offset_f32(timeInput,-mean,timeInput,NFFT);

    status = arm_rfft_fast_init_f32(&S, NFFT);
    arm_rfft_fast_f32(&S,timeInput, frequencyOutput,0);

    if (status != ARM_MATH_SUCCESS){
        LOG_ERR("CMSIS-DSP Calculation Failed");
    }
    
	//remove DC-offset
	frequencyOutput[0] = 0;
	arm_abs_f32(frequencyOutput, frequencyOutput, NFFT);
	arm_max_f32(frequencyOutput,NFFT, &maxValue, &testIndex);
	
	printf("\nMax-Bin: %d, Amplitude %f\r\n",testIndex,maxValue);

	return maxValue;
}

/*Control Vibration Motor*/
static void controlVibration(bool status){

	gpio_pin_set(gpio_dev, 16, status);

}

/*Get Sensor Signal*/
static float32_t processMPU6050()
{
	/*Init Sensor device*/
	const char *const label = DT_LABEL(DT_INST(0, invensense_mpu6050));
	const struct device *mpu6050 = device_get_binding(label);

	if (!mpu6050) {
		LOG_ERR("Failed to find sensor %s", label);
		return;
	}
	LOG_INF("MPU6050 Initialized");

#ifdef CONFIG_MPU6050_TRIGGER
	trigger = (struct sensor_trigger) {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};
	if (sensor_trigger_set(mpu6050, &trigger,
			       handle_mpu6050_drdy) < 0) {
		LOG_ERR("Cannot configure trigger");
		return;
	}
	LOG_INF("Configured for triggered sampling.");
#endif
	
	struct sensor_value temperature;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(mpu6050);

	if (rc == 0) {
		rc = sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(mpu6050, SENSOR_CHAN_GYRO_XYZ,
					gyro);
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
		printk("%f\n",sensor_value_to_double(&accel[2]));
	} else {
		LOG_ERR("sample fetch/get failed: %d", rc);
	}

	return sensor_value_to_double(&accel[2]);
}

/*Control Vibration Motor and Measure*/
static float32_t measureVibration(){

	controlVibration(1);
	float32_t signal = processMPU6050();
	controlVibration(0);

	return signal;
}

/*Run Calibration Mechanism*/
static float32_t calibration(){

	start_calibration = true;
	/*wait for button event --> empty*/
	LOG_WRN("Press Button for Calibration at empty level");
	measureVibration();
	float32_t empty = cmsisFFT();
	
	/*wait for button event --> fill up to half*/
	LOG_WRN("Press Button for Calibration at half level");
	measureVibration();
	float32_t half = cmsisFFT();

	/*wait for button event --> fill up to full*/
	LOG_WRN("Press Button for Calibration at full level");
	measureVibration();
	float32_t full = cmsisFFT();

	return 0;
}

void button_pressed_callback(const struct device *gpiob, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	LOG_WRN("Button pressed");
	button_pressed = true;
	
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
	processMPU6050();

}

#ifdef CONFIG_MPU6050_TRIGGER
static struct sensor_trigger trigger;

static void handle_mpu6050_drdy(const struct device *dev,
				struct sensor_trigger *trig)
{
	int rc = processMPU6050(dev);

	if (rc != 0) {
		LOG_ERR("Resetting trigger due to failure: %d", rc);
		const char *const label = DT_LABEL(DT_INST(0, invensense_mpu6050));
		const struct device *mpu6050 = device_get_binding(label);
		(void)sensor_trigger_set(dev, trig, NULL);
		sensor_trigger_set(mpu6050, &trigger,
			       handle_mpu6050_drdy);
		return;
	}
}
#endif /* CONFIG_MPU6050_TRIGGER */