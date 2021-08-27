#ifndef _PRESSURE_H_
#define _PRESSURE_H_

#include "stdio.h"


volatile uint16_t readings_presssure = 0;
volatile uint16_t Pressure_samples = 25;
volatile uint16_t pressure_avg = 0;
volatile uint16_t Pressure_new_reading = 0;
volatile uint16_t Pressure_last_reading = 0;
volatile uint16_t Pressure_sum = 0;

#endif



