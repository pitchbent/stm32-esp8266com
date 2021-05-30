#include <sensor.h>
uint8_t val_sens(ADC_HandleTypeDef hadc, SENS_STRUCT * sens)
{
	uint8_t percentage;

	sens->an_value = HAL_ADC_GetValue(&hadc);
	if(sens->an_value-sens->wet_value > 0)										//check if the value is positive
	{
		percentage = 100-((sens->wet_value-sens->an_value)*100)/(sens->wet_value-sens->dry_value); // ((an_value-wet)*100) / (dry-wet)
	}
	else
	{
		percentage = 0;
	}
	return percentage;

}


uint8_t cal_sens(ADC_HandleTypeDef hadc, SENS_STRUCT * sens)
{
	uint16_t dry,wet;
	uint8_t cal_counter;

	dry = 0.0;
	wet = 0.0;
	cal_counter = 0;
	sens->timer_flag  = 0;

	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);	//make sure LED is off to avoid confusion


	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_UPDATE);				//clear the update flag so it does not trigger instantly
	HAL_TIM_Base_Start_IT(&htim2);								//wait for 5sec
	while(sens->timer_flag ==0)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	}
	sens->timer_flag = 0; 												//reset timer flag
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


	HAL_ADC_Start_IT(&hadc);									//start the measuring
	while(cal_counter<10)										//measure 10 times
	{
		if(sens->adc_flag == 1)
		{

			cal_counter ++;
			sens->an_value = hadc.Instance->DR;
			dry += sens->an_value;
			sens->adc_flag=0;
			HAL_ADC_Start_IT(&hadc);
		}
	}
	cal_counter = 0;

	dry = dry / 10;												//average value

	sens->dry_value = dry;



	HAL_TIM_Base_Start_IT(&htim2);								//wait 5sec
	while(sens->timer_flag ==0)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
	sens->timer_flag  = 0;




	HAL_TIM_Base_Start_IT(&htim2);								//wait 5sec
	while(sens->timer_flag ==0)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}
	sens->timer_flag  = 0; //reset timer flag
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	HAL_ADC_Start_IT(&hadc);									//measure 10 times
	while(cal_counter<10)
	{
		if(sens->adc_flag == 1)
		{

			cal_counter ++;
			sens->an_value = hadc.Instance->DR;
			wet += sens->an_value;
			sens->adc_flag=0;
			HAL_ADC_Start_IT(&hadc);
		}
	}
	wet = wet / 10;												//average value
	sens->wet_value = wet;

	HAL_ADC_Stop_IT(&hadc);									//stop adc
	sens->adc_flag = 0;										//sanity check


	if((dry-wet)>0)											//check if the values are reasonable
	{
		return 1;											//they do
	}
	else
	{
		return 0;											//they don't
	}




}
