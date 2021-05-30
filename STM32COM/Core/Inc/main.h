/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define VERSION '1'
#define SENSOR 'C' //used sensor


/*Defines DMA */
#define DMA_TIMEOUT_MS 10
#define DMA_BUF_SIZE 128

/*Defines protocol*/
/*Receive Bytes*/
#define STATUS '1'
#define CALIBRATE '2'
#define SEND_VAL '3'
/*Transmit Bytes*/
#define BOARD 'B'
#define ANVALUE 'A'
#define ERROR 'E'
#define ACK 'K'
/*Error Bytes*/
#define ER_UNSPEC '1'	//unspecified error
#define ER_CAL '2'		//calibration error
#define ER_NOT_CAL '3'	//sensor not calibrated
#define ER_UN_MSG '4'	//unknown control message

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define OFF_ASCII 48			// The offset from ASCII Number to decimal is 48
#define OFF_TELE 5				// Complete offset of one telegram, the length of the telegram is length of message + OFF_TELE

typedef struct
{
    volatile uint8_t  t_flag;   // Timeout event flag -> volatile because it's changed during ISR and read in main
    uint8_t tx_flag;			// Flag for Transmission Cplt
    volatile uint8_t av_flag;	// New data is avaiable
    uint16_t timer;             // Timeout duration in msec
    uint16_t prevCOUNT;         // Holds previous value of DMA_COUNT
    uint8_t data[DMA_BUF_SIZE];	// Array for the received Data

} DMA_STRUCT;

typedef struct
{
	uint8_t cal_flag;			// Flag if its calibrated
	uint16_t dry_value;			// Value of the dry sensor, needed for calculation of the percentage
	uint16_t wet_value;			// Value of the wet sensor, "
	volatile uint8_t timer_flag;	// Flag for measuring timer
	uint16_t an_value;			// measured humidity
	uint8_t percentage;			// humidity in percent
	volatile uint8_t adc_flag;	// Flag for ADC complete
	uint8_t send_flag;			// Flag for continuous sending of values

} SENS_STRUCT;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
