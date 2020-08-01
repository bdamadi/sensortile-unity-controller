/**
 ******************************************************************************
 * @file    DataLog/Inc/main.h
 * @author  Central Labs
 * @version V1.1.0
 * @date    27-Sept-2016
 * @brief   This file contains definitions for the main.c file.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_ML_H
#define __MAIN_ML_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "SensorTile.h"
#include "SensorTile_accelero.h"
#include "SensorTile_gyro.h"
#include "SensorTile_magneto.h"
#include "SensorTile_pressure.h"
#include "SensorTile_temperature.h"
#include "SensorTile_humidity.h"
#include "SensorTile_gg.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "embeddedML.h"

	#define STLBLE_PRINTF(...) {\
      char TmpBufferToWrite[256];\
      int32_t TmpBytesToWrite;\
      TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
      CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
    }

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF

#define NUMBER_GESTURES 	6

#define MAX_TRAIN_DATA_CYCLES 	8
#define NUMBER_TRAIN_DATA_CYCLES 1

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern void *LSM6DSM_X_0_handle;
extern void *LSM6DSM_G_0_handle;
extern void *LSM303AGR_X_0_handle;
extern void *LSM303AGR_M_0_handle;

/* Exported functions ------------------------------------------------------- */
void RTC_TimeRegulate( uint8_t hh, uint8_t mm, uint8_t ss );

void LED_Code_Blink(int count);

void CollectTrainData(void *handle, ANN *net, int trainingCycle, int gestureNumber);
int TrainingANN(ANN *net);
int RunANN(void *handle, ANN *net);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_ML_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
