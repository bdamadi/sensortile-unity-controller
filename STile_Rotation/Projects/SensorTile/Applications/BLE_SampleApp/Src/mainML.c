/**
 * MODIFIED BY CHARLES ZALOOM - 8/3/18
 * IMPLEMENTATION OF EMBEDDEDML IN LEARNING DEVICE ORIENTATION
 **/

/**
 ******************************************************************************
 * @file    DataLog/Src/main.c
 * @author  Central Labs
 * @version V1.1.1
 * @date    06-Dec-2016
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <string.h> /* strlen */
#include <stdio.h>  /* sprintf */
#include <math.h>   /* trunc */
#include "embeddedML.h"
#include "mainML.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Data acquisition period [ms] */
#define DATA_PERIOD_MS 10

#define NUMBER_TEST_CYCLES 2
#define CLASSIFICATION_ACC_THRESHOLD 1
#define CLASSIFICATION_DISC_THRESHOLD 1.05
#define START_POSITION_INTERVAL 3000
#define TRAINING_CYCLES 2000
#define LED_BLINK_INTERVAL 200
#define ANGLE_MAG_MAX_THRESHOLD 30
#define MAX_ROTATION_ACQUIRE_CYCLES 2000


//#define NOT_DEBUGGING

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//static volatile uint8_t MEMSInterrupt = 0;
//static volatile uint8_t no_H_HTS221 = 0;
//static volatile uint8_t no_T_HTS221 = 0;
//static volatile uint8_t no_GG = 0;

void *LSM6DSM_X_0_handle = NULL;
void *LSM6DSM_G_0_handle = NULL;
void *LSM303AGR_X_0_handle = NULL;
void *LSM303AGR_M_0_handle = NULL;
//static void *LPS22HB_P_0_handle = NULL;
//static void *LPS22HB_T_0_handle = NULL;
//static void *HTS221_H_0_handle = NULL;
//static void *HTS221_T_0_handle = NULL;
//static void *GG_handle = NULL;

int xyz_initial[3], xyz_initial_prev[3];
int xyz_initial_filter[3], xyz_initial_filter_prev[3];
int xyz_initial_HP[3], xyz_initial_prev_HP[3];
int xyz_initial_filter_HP[3], xyz_initial_filter_prev_HP[3];

float angular_velocity_x_direct;
float angular_velocity_x_direct_filter = 0;
float angular_velocity_x_direct_prev = 0;
float angular_velocity_x_direct_prev_t = 0;
float angular_velocity_x_direct_filter_prev = 0;
float angular_displacement_filter, angular_displacement_filter_prev,
		angular_displacement_prev;

float angular_velocity_x_filter, angular_velocity_x_prev = 0,
		angular_velocity_x_filter_prev = 0;
float angular_displacement = 0;

float training_dataset[NUMBER_GESTURES][MAX_TRAIN_DATA_CYCLES][3];

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

volatile uint8_t hasTrained = 0;
unsigned int training_cycles = TRAINING_CYCLES;

void stable_softmax(float *x, float *y) {
	int size = 3;
	float multiplier = 1.0;

	int i;

	//softmax implemented as square law algorithm to be accommodate numerical precision requirements

	y[0] = (x[0] * x[0] * multiplier)
			/ ((x[0] * x[0] * multiplier) + (x[1] * x[1] * multiplier)
					+ (x[2] * x[2] * multiplier));
	y[1] = (x[1] * x[1] * multiplier)
			/ ((x[0] * x[0] * multiplier) + (x[1] * x[1] * multiplier)
					+ (x[2] * x[2] * multiplier));
	y[2] = (x[2] * x[2] * multiplier)
			/ ((x[0] * x[0] * multiplier) + (x[1] * x[1] * multiplier)
					+ (x[2] * x[2] * multiplier));

	for (i = 0; i < size; i++) {
		if (x[i] < 0.0)
			y[i] = y[i] * -1.0;
	}
}

void motion_softmax(int size, float *x, float *y) {
	float norm;

	norm = sqrt((x[0] * x[0]) + (x[1] * x[1]) + (x[2] * x[2]));
	y[0] = x[0] / norm;
	y[1] = x[1] / norm;
	y[2] = x[2] / norm;
}

void LED_Code_Blink(int count) {

	int i;

	for (i = 0; i < 7; i++) {
		BSP_LED_On(LED1);
		HAL_Delay(30);
		BSP_LED_Off(LED1);
		HAL_Delay(30);
	}

	if (count != 0) {
		HAL_Delay(1000);
		for (i = 0; i < count; i++) {
			BSP_LED_On(LED1);
			HAL_Delay(400);
			BSP_LED_Off(LED1);
			HAL_Delay(400);
		}
	}

	for (i = 0; i < 7; i++) {
		BSP_LED_On(LED1);
		HAL_Delay(30);
		BSP_LED_Off(LED1);
		HAL_Delay(30);
	}

}

void getAccel(void *handle, int *xyz) {
	uint8_t id;
	SensorAxes_t acceleration;
	uint8_t status;

	BSP_ACCELERO_Get_Instance(handle, &id);

	BSP_ACCELERO_IsInitialized(handle, &status);

	if (status == 1) {
		if (BSP_ACCELERO_Get_Axes(handle, &acceleration) == COMPONENT_ERROR) {
			acceleration.AXIS_X = 0;
			acceleration.AXIS_Y = 0;
			acceleration.AXIS_Z = 0;
		}

		xyz[0] = (int) acceleration.AXIS_X;
		xyz[1] = (int) acceleration.AXIS_Y;
		xyz[2] = (int) acceleration.AXIS_Z;
	}
}

void getAngularVelocity(void *handle, int *xyz) {
	uint8_t id;
	SensorAxes_t angular_velocity;
	uint8_t status;

	BSP_GYRO_Get_Instance(handle, &id);
	BSP_GYRO_IsInitialized(handle, &status);

	if (status == 1) {
		if (BSP_GYRO_Get_Axes(handle, &angular_velocity) == COMPONENT_ERROR) {
			angular_velocity.AXIS_X = 0;
			angular_velocity.AXIS_Y = 0;
			angular_velocity.AXIS_Z = 0;
		}
		xyz[0] = (int) angular_velocity.AXIS_X;
		xyz[1] = (int) angular_velocity.AXIS_Y;
		xyz[2] = (int) angular_velocity.AXIS_Z;
	}
}

int Feature_Extraction_Gyro(void *handle, int * ttt_1, int * ttt_2,
		int * ttt_3, int * ttt_mag_scale, int maxAcquiredCycles) {

	int ttt[3], ttt_initial[3], ttt_offset[3];
	int axis_index, sample_index, valid;
	float rotate_angle[3];
	float angle_mag, angle_mag_max_threshold;
	float Tsample;

	angle_mag_max_threshold = ANGLE_MAG_MAX_THRESHOLD;

	/*
	 * Compute sample period with scaling from milliseconds
	 * to seconds
	 */

	Tsample = (float)(DATA_PERIOD_MS)/1000;

	/*
	 * Initialize rotation angle values
	 */

	for (axis_index = 0; axis_index < 3; axis_index++) {
		ttt[axis_index] = 0;
		rotate_angle[axis_index] = 0;
	}

	/*
	 * Rotation Rate Signal integration loop
	 *
	 * Note that loop cycle time is DATA_PERIOD_MS matching the SensorTile
	 * sensor sampling period
	 *
	 * Permit integration loop to operate no longer than a maximum
	 * number of cycles, maxAcquiredCycles. Note: This sets
	 * maximum acquisition time to be maxAcquiredCycles*Tsample
	 *
	 */

	/*
	 * Acquire Rotation Rate values prior to motion
	 *
	 * This includes the initial sensor offset value to be subtracted
	 * from all subsequent samples
	 */

	getAngularVelocity(handle, ttt_offset);

	/*
	 * Notify user to initiate motion
	 */

	BSP_LED_On(LED1);


	valid = 0;
	for (sample_index = 0; sample_index < maxAcquiredCycles; sample_index++) {

		/*
		 * Acquire initial sample value of rotation rate
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt_initial[axis_index] = ttt[axis_index];
		}

		/*
		 * Introduce integration time period delay
		 */

		HAL_Delay(DATA_PERIOD_MS);

		/*
		 * Acquire current sample value of rotation rate and remove
		 * offset value
		 */

		getAngularVelocity(handle, ttt);
		for (axis_index = 0; axis_index < 3; axis_index++) {
			ttt[axis_index] = ttt[axis_index] - ttt_offset[axis_index];
		}


		/*
		 * Suppress value of Z-Axis rotation signals
		 */

//		ttt_initial[2] = 0;
//		ttt[2] = 0;

		/*
		 * Compute rotation angles by integration
		 */

		for (axis_index = 0; axis_index < 3; axis_index++) {
			rotate_angle[axis_index] = rotate_angle[axis_index]
					+ (float)((ttt_initial[axis_index] + ttt[axis_index]) * Tsample / 2);
		}

		/*
		 *
		 *
		 * Compute magnitude of rotational angle summing over X and Y
		 * axis Rotation Rates.
		 *
		 * Convert from milli-degrees to degrees (Note that Rotation
		 * Rate is sampled in milli-degrees per second).
		 *
		 */

		angle_mag = 0;
		for (axis_index = 0; axis_index < 3; axis_index++) {
			angle_mag = angle_mag + pow((rotate_angle[axis_index]), 2);
		}

		/*
		 * Compute angle magnitude and convert from milli-degrees to degrees
		 */

		angle_mag = sqrt(angle_mag)/1000;

		/*
		 * Detect rotation angle magnitude exceeding threshold and exit
		 * integration
		 *
		 * Notify user that angle threshold has been met
		 *
		 */

		if (angle_mag >= angle_mag_max_threshold) {
			valid = 1;
			break;
		}

	}

	/*
	 * Maximum in magnitude found.  Now, compute features
	 * as return values from function.
	 *
	 * 1) Include conversion from milli-degrees to degrees
	 *    (Note that Rotation Rate is sampled in units of
	 *    milli-degrees per second).
	 *
	 * 2) Assign features to rotation angles
	 * 3) Assign 0 to third feature, ttt_3.
	 */


	*ttt_1 = rotate_angle[0] / (1000);
	*ttt_2 = rotate_angle[1] / (1000);
	*ttt_3 = rotate_angle[2] / (1000);

	*ttt_mag_scale = (int) (angle_mag * 100);
	BSP_LED_Off(LED1);
	return valid;
}

void printOutput_ANN(ANN *net, int input_state, int * error) {

	int i, loc, count;
	float point = 0.0;
	float rms_output, mean_output, mean_output_rem, next_max;
	float classification_metric;

	/*
	 * Initialize error state
	 */

	*error = 0;

	loc = -1;
	count = 0;
	mean_output = 0;
	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		mean_output = mean_output + (net->output[i]);
		if (net->output[i] > point && net->output[i] > 0.1) {
			point = net->output[i];
			loc = i;
		}
		count++;
	}

	next_max = 0;
	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		if (i == loc) {
			continue;
		}
		if (net->output[i] > next_max && net->output[i] > 0.1) {
			next_max = net->output[i];
		}
	}

	mean_output = (mean_output) / (count);

	count = 0;
	mean_output_rem = 0;
	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		mean_output_rem = mean_output_rem + (net->output[i]);
		if (i == loc) {
			continue;
		}
		count++;
	}

	mean_output_rem = (mean_output_rem) / (count);

	rms_output = 0;

	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		rms_output = rms_output + pow((net->output[i] - mean_output), 2);
	}

	rms_output = sqrt(rms_output / count);
	if (rms_output != 0) {
		classification_metric = (point - mean_output) / rms_output;
	} else {
		classification_metric = 0;
	}

	if (loc != input_state) {
		rms_output = 0;
		classification_metric = 0;
		point = 0;
		mean_output = 0;
		mean_output_rem = 0;
	}

	STLBLE_PRINTF("\r\nState %i\tMax %i\tMean %i\t\tZ-score %i\tOutputs",
			loc, (int) (100 * point), (int) (100 * mean_output),
			(int) (100 * classification_metric));

	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		STLBLE_PRINTF("\t%i", (int) (100 * net->output[i]));
	}

	if (loc != input_state) {
		*error = 1;
		STLBLE_PRINTF("\t Classification Error");
	}

	if ((loc == input_state)
			&& ((classification_metric < CLASSIFICATION_ACC_THRESHOLD)
					|| ((point / next_max) < CLASSIFICATION_DISC_THRESHOLD))) {
		*error = 1;
		STLBLE_PRINTF("\t Classification Accuracy Limit");
	}

}

int CheckStartPosition(int maxAccquiredCycles)
{
	int ttt[3];
	int i = 0;

	while (1)
	{
		getAccel(LSM6DSM_X_0_handle, ttt);
		if (abs(ttt[0]) < 200 && abs(ttt[1]) < 200 && ttt[2] > 900 && ttt[2] < 1100)
			return 1;

		if (maxAccquiredCycles > 0) {
			i++;
			if (i == maxAccquiredCycles)
				return 0;
		}

//		STLBLE_PRINTF("\r\nAccel Values\t%i\t\%i\t\%i", ttt[0], ttt[1], ttt[2]);
		HAL_Delay(DATA_PERIOD_MS);
	}
}


void CollectTrainData(void *handle, ANN *net, int trainingCycle, int gestureNumber) {

	float XYZ[3];
	float xyz[3];
	int r;
	int ttt_1, ttt_2, ttt_3, ttt_mag_scale;

	STLBLE_PRINTF("\r\nMove to Start Position - Wait for LED On");
	CheckStartPosition(-1); //	HAL_Delay(START_POSITION_INTERVAL);

	STLBLE_PRINTF("\r\nPerform Gesture %d on LED On", gestureNumber + 1);

	Feature_Extraction_Gyro(handle, &ttt_1, &ttt_2, &ttt_3,
			&ttt_mag_scale, MAX_ROTATION_ACQUIRE_CYCLES);

	STLBLE_PRINTF("\r\nAngle Values\t%i\t\%i\t\%i", ttt_1, ttt_2, ttt_3);

	XYZ[0] = (float) ttt_1;
	XYZ[1] = (float) ttt_2;
	XYZ[2] = (float) ttt_3;

	motion_softmax(net->topology[0], XYZ, xyz);

	training_dataset[gestureNumber][trainingCycle][0] = xyz[0];
	training_dataset[gestureNumber][trainingCycle][1] = xyz[1];
	training_dataset[gestureNumber][trainingCycle][2] = xyz[2];

	STLBLE_PRINTF("\r\n Softmax Input \t");
	for (r = 0; r < 3; r++) {
		STLBLE_PRINTF("\t%i", (int) XYZ[r]);
	}
	STLBLE_PRINTF("\r\n Softmax Output\t");
	for (r = 0; r < 3; r++) {
		STLBLE_PRINTF("\t%i", (int) (100 * xyz[r]));
	}
	STLBLE_PRINTF("\r\n\r\n");
}

int TrainingANN(ANN *net) {

	float training_data_init[3];
	float * init_state[NUMBER_GESTURES];
	int i, j, k, m, error, net_error;

	/*
	 * Enter NN training
	 */
	float _Motions[NUMBER_GESTURES][NUMBER_GESTURES] = {
			{ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
			{ 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 },
			{ 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 },
			{ 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 },
			{ 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 },
			{ 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 }
	};

	STLBLE_PRINTF("\r\n\r\nTraining Start\r\n");

	/*
	 * Initialize weights with multistart cycle
	 */
	for (i = 0; i < 6; i++) init_state[i] = _Motions[i];

	init_ann(net);

	/*
	 * Initialize training with default input
	 */

	training_data_init[0] = 1;
	training_data_init[1] = 1;
	training_data_init[2] = 1;
	train_ann(net, training_data_init, init_state[0]);

	for (k = 0; k < NUMBER_TRAIN_DATA_CYCLES; k++) {

		i = 0;
		while (i < training_cycles) {
			for (j = 0; j < NUMBER_GESTURES; j++) {

				if ((i % 20 == 0 && i < 100) || i % 100 == 0) {
					STLBLE_PRINTF("\r\n\r\nTraining Epochs: %d\r\n", i);

					LED_Code_Blink(0);

					net_error = 0;
					for (m = 0; m < NUMBER_GESTURES; m++) {
						run_ann(net, training_dataset[m][k]);
						printOutput_ANN(net, m, &error);
						if (error == 1) {
							net_error = 1;
						}
					}

					STLBLE_PRINTF("\r\nIndex 0 Error State: %i\r\n", net_error);
					if (net_error == 0) {
						return net_error;
					}

				}

				train_ann(net, training_dataset[j][k], _Motions[j]);

				i++;
				HAL_Delay(5);
			}
		}
	}

	return net_error;
}

int RunANN(void *handle, ANN *net) {
	float xyz[3];
	float XYZ[3];
	float point;
	int ttt_1, ttt_2, ttt_3, ttt_mag_scale;
	int i, j, loc, valid;

	BSP_LED_Off(LED1);

	//STLBLE_PRINTF("\n\rMove to Start Position - Wait for LED On");
	//HAL_Delay(START_POSITION_INTERVAL);
	if (!CheckStartPosition(50)) {
		return -2;
	}

	valid = Feature_Extraction_Gyro(handle, &ttt_1, &ttt_2, &ttt_3,
			&ttt_mag_scale, 100);
	if (!valid)
		return -2;

	XYZ[0] = (float) ttt_1;
	XYZ[1] = (float) ttt_2;
	XYZ[2] = (float) ttt_3;

	motion_softmax(net->topology[0], XYZ, xyz);

	STLBLE_PRINTF("\r\n Softmax Input: \t");
	for (j = 0; j < 3; j++) {
		STLBLE_PRINTF("%i\t", (int) XYZ[j]);
	}
	STLBLE_PRINTF("\r\n Softmax Output: \t");
	for (j = 0; j < 3; j++) {
		STLBLE_PRINTF("%i\t", (int) (100 * xyz[j]));
	}
	STLBLE_PRINTF("\r\n");

	run_ann(net, xyz);

	point = 0.0;
	loc = -1;

	for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
		if (net->output[i] > point && net->output[i] > 0.1) {
			point = net->output[i];
			loc = i;
		}
	}

//	if (loc == -1) {
//		LED_Code_Blink(0);
//	} else {
//		LED_Code_Blink(loc + 1);
//	}

	if (loc >= 0 && loc < NUMBER_GESTURES) {
		STLBLE_PRINTF("\n\rNeural Network Classification - Rotation %d", loc);
	} else if (loc == -1) {
		STLBLE_PRINTF("\n\rNeural Network Classification - ERROR");
	} else {
	STLBLE_PRINTF("\n\rNeural Network Classification - NULL");
	}

	return loc;
}

#ifdef __UNUSED_CODE
int Gyro_Sensor_Handler_Rotation(void *handle, ANN *net, int prev_loc) {
	uint8_t id;
	SensorAxes_t angular_velocity;
	uint8_t status;
	float xyz[3];
	float XYZ[3];
	float point;
	int ttt_1, ttt_2, ttt_3, ttt_mag_scale;
	char msg1[128];
	int i, j, k, loc;

	BSP_GYRO_Get_Instance(handle, &id);

	BSP_GYRO_IsInitialized(handle, &status);

	if (status == 1) {
		if (BSP_GYRO_Get_Axes(handle, &angular_velocity) == COMPONENT_ERROR) {
			angular_velocity.AXIS_X = 0;
			angular_velocity.AXIS_Y = 0;
			angular_velocity.AXIS_Z = 0;
		}

		/*
		 * Perform limited number of NN execution and prediction cycles.
		 * Upon return, training will be repeased
		 */

		k = 0;

		while (k < NUMBER_TEST_CYCLES) {

			BSP_LED_Off(LED1);

			sprintf(msg1, "\n\rMove to Start Position - Wait for LED On");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			HAL_Delay(START_POSITION_INTERVAL);

			Feature_Extraction_Gyro(handle, &ttt_1, &ttt_2, &ttt_3,
					&ttt_mag_scale);

            XYZ[0] = (float) ttt_1;
            XYZ[1] = (float) ttt_2;
            XYZ[2] = (float) ttt_3;

			motion_softmax(net->topology[0], XYZ, xyz);

			sprintf(msg1, "\r\n Softmax Input: \t");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			for (j = 0; j < 3; j++) {
				sprintf(msg1, "%i\t", (int) XYZ[j]);
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			}
			sprintf(msg1, "\r\n Softmax Output: \t");
			CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			for (j = 0; j < 3; j++) {
				sprintf(msg1, "%i\t", (int) (100 * xyz[j]));
				CDC_Fill_Buffer((uint8_t *) msg1, strlen(msg1));
			}
			sprintf(msg1, "\r\n");

			run_ann(net, xyz);

			point = 0.0;
			loc = -1;

			for (i = 0; i < net->topology[net->n_layers - 1]; i++) {
				if (net->output[i] > point && net->output[i] > 0.1) {
					point = net->output[i];
					loc = i;
				}
			}

			if (loc == -1) {
				LED_Code_Blink(0);
			} else {
				LED_Code_Blink(loc + 1);
			}

			if (loc >= 0 && loc < 6) {
				STLBLE_PRINTF("\n\rNeural Network Classification - Rotation %d", loc);
			} else if (loc == -1) {
				STLBLE_PRINTF("\n\rNeural Network Classification - ERROR");
			} else {
				STLBLE_PRINTF("\n\rNeural Network Classification - NULL");
			}
			k = k + 1;
		}
	}
	return prev_loc;
}

static int mainML(void) {
	uint32_t msTick, msTickPrev = 0;
	uint8_t doubleTap = 0;
	char msg2[128];

	/* STM32L4xx HAL library initialization:
	 - Configure the Flash prefetch, instruction and Data caches
	 - Configure the Systick to generate an interrupt each 1 msec
	 - Set NVIC Group Priority to 4
	 - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	if (SendOverUSB) {
		/* Initialize LED */
		BSP_LED_Init(LED1);
	}
#ifdef NOT_DEBUGGING
	else
	{
		/* Initialize LEDSWD: Cannot be used during debug because it overrides SWDCLK pin configuration */
		BSP_LED_Init(LEDSWD);
		BSP_LED_Off(LEDSWD);
	}
#endif

	/* enable USB power on Pwrctrl CR2 register */
	HAL_PWREx_EnableVddUSB();

	if (SendOverUSB) /* Configure the USB */
	{
		/*** USB CDC Configuration ***/
		/* Init Device Library */
		USBD_Init(&USBD_Device, &VCP_Desc, 0);
		/* Add Supported Class */
		USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
		/* Add Interface callbacks for AUDIO and CDC Class */
		USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
		/* Start Device Process */
		USBD_Start(&USBD_Device);
	} else /* Configure the SDCard */
	{
		DATALOG_SD_Init();
	}
	HAL_Delay(200);

	/* Configure and disable all the Chip Select pins */
	Sensor_IO_SPI_CS_Init_All();

	/* Initialize and Enable the available sensors */
	initializeAllSensors();
	enableAllSensors();

	sprintf(msg2, "\n\rEmbeddedML Rotation Angle Classification");
	CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));

	/* Notify user */

	sprintf(msg2, "\n\rDouble Tap to start training");
	CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));


	//---EMBEDDED ANN---
	float weights[81] = { 0.680700, 0.324900, 0.607300, 0.365800, 0.693000,
			0.527200, 0.754400, 0.287800, 0.592300, 0.570900, 0.644000,
			0.416500, 0.249200, 0.704200, 0.598700, 0.250300, 0.632700,
			0.372900, 0.684000, 0.661200, 0.230300, 0.516900, 0.770900,
			0.315700, 0.756000, 0.293300, 0.509900, 0.627800, 0.781600,
			0.733500, 0.509700, 0.382600, 0.551200, 0.326700, 0.781000,
			0.563300, 0.297900, 0.714900, 0.257900, 0.682100, 0.596700,
			0.467200, 0.339300, 0.533600, 0.548500, 0.374500, 0.722800,
			0.209100, 0.619400, 0.635700, 0.300100, 0.715300, 0.670800,
			0.794400, 0.766800, 0.349000, 0.412400, 0.619600, 0.353000,
			0.690300, 0.772200, 0.666600, 0.254900, 0.402400, 0.780100,
			0.285300, 0.697700, 0.540800, 0.222800, 0.693300, 0.229800,
			0.698100, 0.463500, 0.201300, 0.786500, 0.581400, 0.706300,
			0.653600, 0.542500, 0.766900, 0.411500 };
	float dedw[81];
	float bias[15];
	unsigned int network_topology[3] = { 3, 9, 6 };
	float output[6];

	ANN net;
	net.weights = weights;
	net.dedw = dedw;
	net.bias = bias;
	net.topology = network_topology;
	net.n_layers = 3;
	net.n_weights = 81;
	net.n_bias = 15;
	net.output = output;

	//OPTIONS
	net.eta = 0.13;     //Learning Rate
	net.beta = 0.01;    //Bias Learning Rate
	net.alpha = 0.25;   //Momentum Coefficient
	net.output_activation_function = &relu2;
	net.hidden_activation_function = &relu2;

	init_ann(&net);
	//---------------------

	int loc = -1;
	while (1) {
		/* Get sysTick value and check if it's time to execute the task */
		msTick = HAL_GetTick();
		if (msTick % DATA_PERIOD_MS == 0 && msTickPrev != msTick) {
			msTickPrev = msTick;

			if (SendOverUSB) {
				BSP_LED_On(LED1);
			}

			//RTC_Handler( &RtcHandle );

			if (hasTrained) {
				loc = Gyro_Sensor_Handler_Rotation(LSM6DSM_G_0_handle, &net,
						loc);
				/*
				 * Upon return from Accel_Sensor_Handler, initiate retraining.
				 */
				hasTrained = 0;
				sprintf(msg2,
						"\n\r\n\rDouble Tap to start a new training session");
				CDC_Fill_Buffer((uint8_t *) msg2, strlen(msg2));
			}

			if (SendOverUSB) {
				BSP_LED_Off(LED1);

			}

		}

		/* Check LSM6DSM Double Tap Event  */
		if (!hasTrained) {
			BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(LSM6DSM_X_0_handle,&doubleTap);
		if (doubleTap) {
			LED_Code_Blink(0);
			TrainRotation(LSM6DSM_G_0_handle, &net);
			hasTrained = 1;
			}
		}

		/* Go to Sleep */
		__WFI();
	}
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
void initializeAllSensors(void) {
	if (BSP_ACCELERO_Init(LSM6DSM_X_0, &LSM6DSM_X_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_GYRO_Init(LSM6DSM_G_0, &LSM6DSM_G_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_ACCELERO_Init(LSM303AGR_X_0, &LSM303AGR_X_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_MAGNETO_Init(LSM303AGR_M_0, &LSM303AGR_M_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_PRESSURE_Init(LPS22HB_P_0, &LPS22HB_P_0_handle) != COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_TEMPERATURE_Init(LPS22HB_T_0, &LPS22HB_T_0_handle)
			!= COMPONENT_OK) {
		while (1)
			;
	}

	if (BSP_TEMPERATURE_Init(HTS221_T_0, &HTS221_T_0_handle)
			== COMPONENT_ERROR) {
		no_T_HTS221 = 1;
	}

	if (BSP_HUMIDITY_Init(HTS221_H_0, &HTS221_H_0_handle) == COMPONENT_ERROR) {
		no_H_HTS221 = 1;
	}

	/* Inialize the Gas Gauge if the battery is present */
	if (BSP_GG_Init(&GG_handle) == COMPONENT_ERROR) {
		no_GG = 1;
	}

	//if(!SendOverUSB)
	//{
	/* Enable HW Double Tap detection */
	BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(LSM6DSM_X_0_handle);
	BSP_ACCELERO_Set_Tap_Threshold_Ext(LSM6DSM_X_0_handle,
	LSM6DSM_TAP_THRESHOLD_MID);
	//}

}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void enableAllSensors(void) {
	BSP_ACCELERO_Sensor_Enable(LSM6DSM_X_0_handle);
	BSP_GYRO_Sensor_Enable(LSM6DSM_G_0_handle);
	BSP_ACCELERO_Sensor_Enable(LSM303AGR_X_0_handle);
	BSP_MAGNETO_Sensor_Enable(LSM303AGR_M_0_handle);
	BSP_PRESSURE_Sensor_Enable(LPS22HB_P_0_handle);
	BSP_TEMPERATURE_Sensor_Enable(LPS22HB_T_0_handle);
	if (!no_T_HTS221) {
		BSP_TEMPERATURE_Sensor_Enable(HTS221_T_0_handle);
		BSP_HUMIDITY_Sensor_Enable(HTS221_H_0_handle);
	}

}

/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
void disableAllSensors(void) {
	BSP_ACCELERO_Sensor_Disable(LSM6DSM_X_0_handle);
	BSP_ACCELERO_Sensor_Disable(LSM303AGR_X_0_handle);
	BSP_GYRO_Sensor_Disable(LSM6DSM_G_0_handle);
	BSP_MAGNETO_Sensor_Disable(LSM303AGR_M_0_handle);
	BSP_HUMIDITY_Sensor_Disable(HTS221_H_0_handle);
	BSP_TEMPERATURE_Sensor_Disable(HTS221_T_0_handle);
	BSP_TEMPERATURE_Sensor_Disable(LPS22HB_T_0_handle);
	BSP_PRESSURE_Sensor_Disable(LPS22HB_P_0_handle);
}
#endif

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{

	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	while (1)
	{}
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

