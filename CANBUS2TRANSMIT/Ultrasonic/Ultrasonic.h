#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include "stdio.h"

volatile uint16_t readings_ultrasonic = 0;
volatile uint16_t ultrasonic_samples = 25;
volatile uint16_t ultrasonic_avg = 0;
volatile uint16_t ulrtrasonic_new_reading = 0;
volatile uint16_t ultrasonic_last_reading = 0;
volatile uint16_t ultrasonic_sum = 0;


#endif



