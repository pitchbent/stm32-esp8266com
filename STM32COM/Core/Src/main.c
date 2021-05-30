/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "xtea.h"
#include "crc16ccitt.h"
#include "uartcom.h"
#include "sensor.h"


#include <stdbool.h> //There is no real true or false anymore
#include <stdio.h>	 //sprintf
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */



DMA_STRUCT dma_info = {0,0,0,DMA_TIMEOUT_MS,DMA_BUF_SIZE,{'\0'}};
SENS_STRUCT sensor = {0,0,0,0};


/*Variables UART*/
uint8_t dma_rx_buf[DMA_BUF_SIZE];       /* Circular buffer for DMA */
//uint8_t data[DMA_BUF_SIZE] = {'\0'};    /* Data buffer that contains newly received data -*/


//uint8_t err_msg[4] = {'e','r','r','\n'};
uint8_t TxBuffer[10];				//Sending buffer
uint8_t TxLen;						//Length of the data to be sent
//uint8_t SensorValue = 's';
//char Filler = 'x';
//char startMaker = '<';
uint8_t data_len;
uint16_t test_array[DMA_BUF_SIZE];

/*Variables ADC*/
uint16_t adc_value;
uint8_t adc_complete_flag;
//uint8_t adc_send_buffer[5];

/*Variables Timer*/
volatile uint8_t timer1_f; //flags for timers
volatile uint8_t timer2_f;


/*Variables for XTEA*/
uint32_t xt_data[2] = {0x266e817d,0xbacd5035}; //2*32Bit Data
uint32_t xt_key[4] = {KEY1,KEY2,KEY3,KEY4};	  //128Bit Key


uint8_t Test[6] = {0x3c,0x30,0x31,0x30,0xa6,0x99};
uint16_t return_crc_test = 0;
double var,var1,var2 = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Prototypes for Callbacks
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);	//Calibrate ADC
  //HAL_ADC_Start_IT(&hadc1);

  //HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim1);




  __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);			//Enable Idle Line Interrupt

  if(HAL_UART_Receive_DMA(&huart3, dma_rx_buf, DMA_BUF_SIZE)==HAL_ERROR)			//Catch possible fault
  {
	  Error_Handler();
  }






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(sensor.adc_flag == 1)
	  {
		  if (sensor.cal_flag == 1)
		  {
			  sensor.percentage = val_sens(hadc1, &sensor);	//read sensor and convert to %
			  sensor.adc_flag = 0;
			  //sensor.an_value = HAL_ADC_GetValue(&hadc1);
			  //if(sensor.an_value-sensor.wet_value > 0)										//check if the value is positive
			  //{
			  //sensor.percentage = 100-((sensor.wet_value-sensor.an_value)*100)/(sensor.wet_value-sensor.dry_value); // ((an_value-wet)*100) / (dry-wet)
			  //}
			  //else
			  //{
			  //sensor.percentage = 0;
			  //}
			  TxLen = an_send(sensor.percentage, TxBuffer);
			  HAL_UART_Transmit_DMA(&huart3, TxBuffer, TxLen);
			  tx_wait(&dma_info);
		  }
		  else
		  {
			  TxLen = er_send(ER_CAL,TxBuffer);
			  HAL_UART_Transmit_DMA(&huart3, TxBuffer, TxLen);
			  tx_wait(&dma_info);
		  }


		  //xt_data[0] = (SensorValue<<24 | adc_send_buffer[1]<<16 | adc_send_buffer[0]<<8 | Filler);  //Concatenate Hex input to one uint32t
		  //xt_data[1] = (Filler<<24 | Filler<<16 | Filler<<8 | Filler);

		  //Encrypt_XTEA(xt_data, xt_key);
//		  sprintf(adc_send_buffer,"%d",adc_value);
//		  crc16_calc = CRC16_buf(adc_send_buffer, 5);







		  adc_complete_flag = 0;
	  }



	  if(dma_info.av_flag == 1)
	  {
		  //sensor.calibrated =1;
		  //sensor.percentage=100;
		  dma_info.av_flag = 0;
		  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  data_len = data_check(dma_info.data);

		  if (data_len != 0) {
			  switch (dma_info.data[3]) {


				case STATUS:
					TxLen = ack_send(STATUS, TxBuffer);
					HAL_UART_Transmit_DMA(&huart3, TxBuffer, TxLen);
					tx_wait(&dma_info);

					TxLen = stat_send(TxBuffer);
					HAL_UART_Transmit_DMA(&huart3, TxBuffer, TxLen);
					tx_wait(&dma_info);
					break;


				case CALIBRATE:
					TxLen = ack_send(CALIBRATE, TxBuffer);
					HAL_UART_Transmit_DMA(&huart3, TxBuffer, TxLen);
					tx_wait(&dma_info);

					sensor.cal_flag = cal_sens(hadc1,&sensor);
					if (sensor.cal_flag == 0)					//Sensor was not calibrated
					{
						TxLen = er_send(ER_CAL,TxBuffer);
						HAL_UART_Transmit_DMA(&huart3, TxBuffer, TxLen);
						tx_wait(&dma_info);
					}

					break;


				case SEND_VAL:
					TxLen = ack_send(SEND_VAL, TxBuffer);
					HAL_UART_Transmit_DMA(&huart3, TxBuffer, TxLen);
					tx_wait(&dma_info);

					if (dma_info.data[4] == '1')
					{
						if(sensor.cal_flag == 1)
						{
							//enable timer for adc
							HAL_TIM_Base_Start_IT(&htim1);
						}
						else
						{
							TxLen = er_send(ER_NOT_CAL,TxBuffer);
							HAL_UART_Transmit_DMA(&huart3, TxBuffer, TxLen);
							tx_wait(&dma_info);
						}
					}
					else
					{
						//disable timer for adc
						HAL_TIM_Base_Stop_IT(&htim1);
					}
					break;

				default:
					TxLen = er_send(ER_UN_MSG,TxBuffer);
					HAL_UART_Transmit_DMA(&huart3, TxBuffer, TxLen);
					tx_wait(&dma_info);

					break;
			}

		}





			  //snprintf(TxBuffer, sizeof(TxBuffer),"%d", adc_value);

			  //HAL_UART_Transmit_DMA(&huart3, TxBuffer, sizeof(TxBuffer));

			  //xt_data[0] = (data[1]<<24 | data[2]<<16 | data[3]<<8 | data[4]);  //Concatenate Hex input to one uint32t
			  //xt_data[1] = (data[5]<<24 | data[6]<<16 | data[7]<<8 | data[8]);
			  //Decrypt_XTEA(xt_data, xt_key);

//			  for (int i = 0; i < sizeof(data); ++i) {
//				data[i] = 0;
			}

//		  else
//		  {
//			  HAL_UART_Transmit_DMA(&huart3, err_msg, 4);
//		  }
//	  }



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*Timer Callback*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	/*Check which timer isr fired*/
	if(htim->Instance == TIM1)
	{
		HAL_ADC_Start_IT(&hadc1);
	}
	if(htim->Instance == TIM2)
	{
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_TIM_Base_Stop_IT(&htim2);
		sensor.timer_flag= 1;
	}
}


/*ADC Callback*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	HAL_ADC_Stop_IT(&hadc1);
	sensor.adc_flag = 1;
}


/*Callback for a full receive buffer*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t  pos, start, length;
    uint16_t currCOUNT = __HAL_DMA_GET_COUNTER(huart->hdmarx);




    /* Ignore timeout flag if the buffer is perfectly filled but there was no new data until timeout*/
    if(dma_info.t_flag && currCOUNT == DMA_BUF_SIZE)
    {
        dma_info.t_flag = 0;
        return;
    }


    /* Calc the start value based on the length of the previous data */
    if(dma_info.prevCOUNT<DMA_BUF_SIZE)
    {
    	start = (DMA_BUF_SIZE-dma_info.prevCOUNT);
    }
    else
    {
    	start = 0;
    }


    if(dma_info.t_flag)    /* Timeout event */
    {
        /* Calc the length data */

    	if(dma_info.prevCOUNT < DMA_BUF_SIZE)
    	{
    		length = dma_info.prevCOUNT - currCOUNT; //There is old data to be ignored
    	}
    		else
    		{
    			length = DMA_BUF_SIZE - currCOUNT;  //Everything is new until the current position
    		}
		dma_info.prevCOUNT = currCOUNT;				//pass current position
        dma_info.t_flag = 0;
    }
    	else                /* DMA Rx Complete event -> the entire buffer is full */
    	{
    		length = DMA_BUF_SIZE - start;
    		dma_info.prevCOUNT = DMA_BUF_SIZE;
    	}

    /* Copy and Process new data */
    for(uint16_t i=0,pos=start; i<length; ++i,++pos)
    {
        dma_info.data[i] = dma_rx_buf[pos];
    }
    dma_info.av_flag = 1;

    /*Bounce data back if the previous msg was sent out*/
//    if(dma_info.tx_flag == 0)
//    {
//    	HAL_UART_Transmit_DMA(&huart3, data, length);
//    	dma_info.tx_flag = 1;
//    }



}


/*Callback when Transmission was 1/2 completed*/
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huar)
{

}
/*Callback when transmission was completed*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	dma_info.tx_flag = 1;
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
