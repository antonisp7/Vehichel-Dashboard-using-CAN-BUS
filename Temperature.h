#ifndef _TEMPERATURE_H_
#define _TEMPERATURE_H_

#include "stdio.h"

volatile uint16_t readings_temp = 0;
volatile uint16_t Tpemperature_samples = 25;
volatile uint16_t Temperature_avg = 0;
volatile uint16_t Temperature_new_reading = 0;
volatile uint16_t Temperature_last_reading = 0;
volatile uint16_t Temperature_sum = 0;


#endif



