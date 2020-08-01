/**
 ******************************************************************************
 * @file    main.c
 * @author  Central LAB
 * @version V1.0.0
 * @date    21-Nov-2016
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
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
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "TargetFeatures.h"
#include "main.h"
#include "mainML.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Data acquisition period [ms] */
#define DATA_PERIOD_MS 10

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;

extern TIM_HandleTypeDef TimHandle;
extern void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* Exported Variables -------------------------------------------------------------*/

uint32_t ConnectionBleStatus = 0;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

TIM_HandleTypeDef TimCCHandle;

uint8_t bdaddr[6];

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t HCI_ProcessEvent = 0;
static volatile uint32_t SendEnv = 0;

USBD_HandleTypeDef USBD_Device;

/* Private function prototypes -----------------------------------------------*/
static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);
static void InitTimers(void);

//uint8_t CDC_Fill_Buffer(uint8_t* Buf, uint32_t TotalLen){}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
	uint32_t msTick, msTickPrev = 0;
	uint8_t doubleTap = 0;

	/* STM32L4xx HAL library initialization:
	 - Configure the Flash prefetch, instruction and Data caches
	 - Configure the Systick to generate an interrupt each 1 msec
	 - Set NVIC Group Priority to 4
	 - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();

	/* Configure the System clock */
	SystemClock_Config();

	/* Configure and disable all the Chip Select pins */
	Sensor_IO_SPI_CS_Init_All();

	InitTargetPlatform(TARGET_SENSORTILE);

	STLBLE_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n"
			"\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
			" (IAR)\r\n"
#elif defined (__CC_ARM)
			" (KEIL)\r\n"
#elif defined (__GNUC__)
			" (openstm32)\r\n"
#endif
			"\tSend Every %4dmS Temperature/Humidity/Pressure\r\n",
			HAL_GetHalVersion() >> 24, (HAL_GetHalVersion() >> 16) & 0xFF,
			(HAL_GetHalVersion() >> 8) & 0xFF, HAL_GetHalVersion() & 0xFF,
			__DATE__, __TIME__, uhCCR1_Val/10);

#ifdef ENABLE_USB_DEBUG_CONNECTION
	STLBLE_PRINTF("Debug Connection         Enabled\r\n");
#endif /* ENABLE_USB_DEBUG_CONNECTION */

#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
	STLBLE_PRINTF("Debug Notify Trasmission Enabled\r\n");
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */

	/* Initialize the BlueNRG */
	Init_BlueNRG_Stack();

	/* Initialize the BlueNRG Custom services */
	Init_BlueNRG_Custom_Services();

	/* initialize timers */
	InitTimers();

	/* Code from the ML project */

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
	unsigned int network_topology[3] = { 3, 9, NUMBER_GESTURES };
	float output[NUMBER_GESTURES];

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
	/* End of Code from the ML project */

	int loc = -1, i;

	volatile uint8_t hasTrained = 0;
	volatile uint8_t trainedGestureDataCycle[NUMBER_GESTURES] = { 0, 0, 0, 0, 0, 0 };

	STLBLE_PRINTF("\r\nEmbeddedML Rotation Angle Classification\r\n");
	STLBLE_PRINTF("Waiting training request to start training\r\n");

	/* Infinite loop */

	while (1) {
		/* handle BLE event */
		if (HCI_ProcessEvent) {
			HCI_ProcessEvent = 0;
			HCI_Process();
		}

		/* Update the BLE advertise data and make the Board connectable */
		if (set_connectable) {
			setConnectable();
			set_connectable = FALSE;
		}

		/* Get sysTick value and check if it's time to execute the task */
		msTick = HAL_GetTick();
		if (msTick % DATA_PERIOD_MS == 0 && msTickPrev != msTick) {
			msTickPrev = msTick;

			BSP_LED_On(LED1);

			//RTC_Handler( &RtcHandle );

			if (hasTrained) {
				loc = RunANN(LSM6DSM_G_0_handle, &net);
				if (loc >= 0 && loc < NUMBER_GESTURES) {
					/* Send motion data to BLE connection */
					Motion_Update(hasTrained, loc, trainedGestureDataCycle);
				}
			}

			BSP_LED_Off(LED1);
		}

		// Check training request
		int trainingGesture = -1;
		if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_1)) {
			trainingGesture = 0;
			W2ST_OFF_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_1);
		} else if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_2)) {
			trainingGesture = 1;
			W2ST_OFF_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_2);
		} else if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_3)) {
			trainingGesture = 2;
			W2ST_OFF_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_3);
		} else if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_4)) {
			trainingGesture = 3;
			W2ST_OFF_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_4);
		} else if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_5)) {
			trainingGesture = 4;
			W2ST_OFF_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_5);
		} else if (W2ST_CHECK_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_6)) {
			trainingGesture = 5;
			W2ST_OFF_CONNECTION(W2ST_CONNECT_TRAINING_MOTION_6);
		}

		if (trainingGesture >= 0) {
			if (hasTrained) {
				/* initiate retraining */
				hasTrained = 0;
				for (i = 0; i < NUMBER_GESTURES; i++) {
					trainedGestureDataCycle[i] = 0;
				}
				STLBLE_PRINTF("\n\r\n\rStart a new training session\r\n");
			}

			BSP_LED_Off(LED1);
			HAL_Delay(2000);

			// Collect data for training the specified gesture */
			if (trainedGestureDataCycle[trainingGesture] >= NUMBER_TRAIN_DATA_CYCLES) {
				trainedGestureDataCycle[trainingGesture] = 0;
			}
			CollectTrainData(LSM6DSM_G_0_handle, &net, trainedGestureDataCycle[trainingGesture], trainingGesture);
			trainedGestureDataCycle[trainingGesture]++;

			/* Send training status to BLE connection */
			Motion_Update(hasTrained, -1, trainedGestureDataCycle);

			// Check if all gestures have been trained
			hasTrained = 1;
			for (i = 0; i < NUMBER_GESTURES; i++) {
				if (trainedGestureDataCycle[i] < NUMBER_TRAIN_DATA_CYCLES) {
					hasTrained = 0;
					break;
				}
			}

			if (hasTrained) {
				/* Send training status to BLE connection */
				Motion_Update(-1, -1, trainedGestureDataCycle);

				// Finish collecting data, start training
				if (TrainingANN(&net) == 0) {
					LED_Code_Blink(0);
					LED_Code_Blink(0);
				} else {
					LED_Code_Blink(1);
					LED_Code_Blink(1);
				}

				STLBLE_PRINTF("\r\n\r\nTraining Complete, Now Start Detecting Motions\r\n");

				/* Send training status to BLE connection */
				Motion_Update(hasTrained, -1, trainedGestureDataCycle);
			}
		}

		/* Wait for Interrupt */
		__WFI();
	}
}

/**
 * @brief  Output Compare callback in non blocking mode
 * @param  htim : TIM OC handle
 * @retval None
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	uint32_t uhCapture = 0;

	/* TIM1_CH1 toggling with frequency = 2Hz */
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		/* Set the Capture Compare Register value */
		__HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1,
				(uhCapture + uhCCR1_Val));
		SendEnv = 1;
	}
}

/**
 * @brief  Period elapsed callback in non blocking mode for Environmental timer
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == (&TimHandle)) {
		CDC_TIM_PeriodElapsedCallback(htim);
	}
}

#ifdef __UNUSED_CODE
/**
 * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
 * @param  None
 * @retval None
 */
static void SendEnvironmentalData(void)
{
	uint8_t Status;

#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
	if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
		BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ");
		Term_Update(BufferToWrite,BytesToWrite);
	} else {
		STLBLE_PRINTF("Sending: ");
	}
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */

	/* Pressure,Humidity, and Temperatures*/
	if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV)) {
		float SensorValue;
		int32_t PressToSend=0;
		uint16_t HumToSend=0;
		int16_t Temp2ToSend=0,Temp1ToSend=0;
		int32_t decPart, intPart;

		if(TargetBoardFeatures.HandlePressSensor) {
			if(BSP_PRESSURE_IsInitialized(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK) {
				BSP_PRESSURE_Get_Press(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValue);
				MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
				PressToSend=intPart*100+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
				if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
					BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
					Term_Update(BufferToWrite,BytesToWrite);
				} else {
					STLBLE_PRINTF("Press=%ld ",PressToSend);
				}
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
			}
		}

		if(TargetBoardFeatures.HandleHumSensor) {
			if(BSP_HUMIDITY_IsInitialized(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK) {
				BSP_HUMIDITY_Get_Hum(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValue);
				MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
				HumToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
				if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
					BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
					Term_Update(BufferToWrite,BytesToWrite);
				} else {
					STLBLE_PRINTF("Hum=%d ",HumToSend);
				}
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
			}
		}

		if(TargetBoardFeatures.NumTempSensors==2) {
			if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK) {
				BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
				MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
				Temp1ToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
				if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
					BytesToWrite = sprintf((char *)BufferToWrite,"Temp=%d ",Temp1ToSend);
					Term_Update(BufferToWrite,BytesToWrite);
				} else {
					STLBLE_PRINTF("Temp=%d ",Temp1ToSend);
				}
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
			}

			if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[1],&Status)==COMPONENT_OK) {
				BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[1],(float *)&SensorValue);
				MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
				Temp2ToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
				if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
					BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
					Term_Update(BufferToWrite,BytesToWrite);
				} else {
					STLBLE_PRINTF("Temp2=%d ",Temp2ToSend);
				}
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
			}
		} else if(TargetBoardFeatures.NumTempSensors==1) {
			if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK) {
				BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
				MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
				Temp1ToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
				if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
					BytesToWrite = sprintf((char *)BufferToWrite,"Temp1=%d ",Temp1ToSend);
					Term_Update(BufferToWrite,BytesToWrite);
				} else {
					STLBLE_PRINTF("Temp1=%d ",Temp1ToSend);
				}
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
			}
		}
		Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
	}

#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
	if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
		BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
		Term_Update(BufferToWrite,BytesToWrite);
	} else {
		STLBLE_PRINTF("\r\n");
	}
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
}

#endif /* __UNUSED_CODE */

/**
 * @brief  Function for initializing timers for sending the information to BLE:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void InitTimers(void) {
	uint32_t uwPrescalerValue;

	/* Timer Output Compare Configuration Structure declaration */
	TIM_OC_InitTypeDef sConfig;

	/* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
	uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

	/* Set TIM1 instance (Motion)*/
	/* Set TIM1 instance */
	TimCCHandle.Instance = TIM1;
	TimCCHandle.Init.Period = 65535;
	TimCCHandle.Init.Prescaler = uwPrescalerValue;
	TimCCHandle.Init.ClockDivision = 0;
	TimCCHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if (HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/* Configure the Output Compare channels */
	/* Common configuration for all channels */
	sConfig.OCMode = TIM_OCMODE_TOGGLE;
	sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

	/* Output Compare Toggle Mode configuration: Channel1 */
	sConfig.Pulse = uhCCR1_Val;
	if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1)
			!= HAL_OK) {
		/* Configuration Error */
		Error_Handler();
	}

}

/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(void) {
	const char BoardName[8] = { NAME_STLBLE, 0 };
	uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
	int ret;
	uint8_t hwVersion;
	uint16_t fwVersion;

#ifdef STATIC_BLE_MAC
	{
		uint8_t tmp_bdaddr[6]= {STATIC_BLE_MAC};
		int32_t i;
		for(i=0;i<6;i++)
		bdaddr[i] = tmp_bdaddr[i];
	}
#endif /* STATIC_BLE_MAC */

	/* Initialize the BlueNRG SPI driver */
	BNRG_SPI_Init();

	/* Initialize the BlueNRG HCI */
	HCI_Init();

	/* Reset BlueNRG hardware */
	BlueNRG_RST();

	/* get the BlueNRG HW and FW versions */
	getBlueNRGVersion(&hwVersion, &fwVersion);

	if (hwVersion > 0x30) {
		/* X-NUCLEO-IDB05A1 expansion board is used */
		TargetBoardFeatures.bnrg_expansion_board = IDB05A1;
	} else {
		/* X-NUCLEO-IDB0041 expansion board is used */
		TargetBoardFeatures.bnrg_expansion_board = IDB04A1;
	}

	/*
	 * Reset BlueNRG again otherwise it will fail.
	 */
	BlueNRG_RST();

#ifndef STATIC_BLE_MAC
	/* Create a Unique BLE MAC */
	{
		bdaddr[0] = (STM32_UUID[1] >> 24) & 0xFF;
		bdaddr[1] = (STM32_UUID[0]) & 0xFF;
		bdaddr[2] = (STM32_UUID[2] >> 8) & 0xFF;
		bdaddr[3] = (STM32_UUID[0] >> 16) & 0xFF;
		bdaddr[4] = (((STLBLE_VERSION_MAJOR - 48) * 10)
				+ (STLBLE_VERSION_MINOR - 48) + 100) & 0xFF;
		bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
	}
#else /* STATIC_BLE_MAC */

	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
			CONFIG_DATA_PUBADDR_LEN,
			bdaddr);
	if(ret) {
		STLBLE_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
		goto fail;
	}
#endif /* STATIC_BLE_MAC */

	ret = aci_gatt_init();
	if (ret) {
		STLBLE_PRINTF("\r\nGATT_Init failed\r\n");
		goto fail;
	}

	if (TargetBoardFeatures.bnrg_expansion_board == IDB05A1) {
		ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07,
				&service_handle, &dev_name_char_handle,
				&appearance_char_handle);
	} else {
		ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle,
				&dev_name_char_handle, &appearance_char_handle);
	}

	if (ret != BLE_STATUS_SUCCESS) {
		STLBLE_PRINTF("\r\nGAP_Init failed\r\n");
		goto fail;
	}

#ifndef  STATIC_BLE_MAC
	ret = hci_le_set_random_address(bdaddr);

	if (ret) {
		STLBLE_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
		goto fail;
	}
#endif /* STATIC_BLE_MAC */

	ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
			7/*strlen(BoardName)*/, (uint8_t *) BoardName);

	if (ret) {
		STLBLE_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
		while (1)
			;
	}

	ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
	OOB_AUTH_DATA_ABSENT,
	NULL, 7, 16,
	USE_FIXED_PIN_FOR_PAIRING, 123456,
	BONDING);
	if (ret != BLE_STATUS_SUCCESS) {
		STLBLE_PRINTF("\r\nGAP setting Authentication failed\r\n");
		goto fail;
	}

	STLBLE_PRINTF("SERVER: BLE Stack Initialized \r\n"
			"\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
			"\t\tBoardName= %s\r\n"
			"\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n", "SensorTile", hwVersion,
			fwVersion >> 8, (fwVersion >> 4) & 0xF,
			(hwVersion > 0x30) ? ('a' + (fwVersion & 0xF) - 1) : 'a', BoardName,
			bdaddr[5], bdaddr[4], bdaddr[3], bdaddr[2], bdaddr[1], bdaddr[0]);

	/* Set output power level */
	aci_hal_set_tx_power_level(1, 4);

	return;

	fail: return;
}

/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void) {
	int ret;

	ret = Add_HWServW2ST_Service();
	if (ret == BLE_STATUS_SUCCESS) {
		STLBLE_PRINTF("HW      Service W2ST added successfully\r\n");
	} else {
		STLBLE_PRINTF("\r\nError while adding HW Service W2ST\r\n");
	}

	ret = Add_ConfigW2ST_Service();
	if (ret == BLE_STATUS_SUCCESS) {
		STLBLE_PRINTF("Config  Service W2ST added successfully\r\n");
	} else {
		STLBLE_PRINTF("\r\nError while adding Config Service W2ST\r\n");
	}
}

/**
 * @brief This function provides accurate delay (in milliseconds) based
 *        on variable incremented.
 * @note This is a user implementation using WFI state
 * @param Delay: specifies the delay time length, in milliseconds.
 * @retval None
 */
void HAL_Delay(__IO uint32_t Delay) {
	uint32_t tickstart = 0;
	tickstart = HAL_GetTick();
	while ((HAL_GetTick() - tickstart) < Delay) {
		__WFI();
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* User may add here some code to deal with this error */
	while (1) {
	}
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case BNRG_SPI_EXTI_PIN:
		HCI_Isr();
		HCI_ProcessEvent = 1;
		break;
	}
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: STLBLE_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1) {
	}
}
#endif

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
