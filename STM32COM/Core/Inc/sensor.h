#ifndef __SENSOR_H
#define __SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <main.h>

extern uint8_t adc_complete_flag;	//link used variables
extern TIM_HandleTypeDef htim2;
//extern SENS_STRUCT sensor;
extern volatile uint8_t timer2_f;


/*This function calibrates the sensor. Needs adc handler aswell as the sensor struct
 *LED on: 	sensor needs to be dry
 *LED off: 	measuring 10 cycles
 *LED on: 	sensor needs to be soaked
 *LED off:	measuring 10 cycles
 *LED on:	measuring done
 *
 *Returns 1 if calibration was succesfull, 0 if not. Value is stored in sens.coeff*/

uint8_t cal_sens(ADC_HandleTypeDef hadc, SENS_STRUCT * sens);


#ifdef __cplusplus
}
#endif

#endif
