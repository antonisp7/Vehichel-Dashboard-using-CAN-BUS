/*
 * File: DC_MOTOR_cfg.c
 * Driver Name: [[ DC MOTOR ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "PWM.h"

const PWM_CfgType PWM_CfgParam[PWM_UNITS] =
{
	// 1 Configurations
    {
	    GPIOB,
		GPIOB,
		GPIO_PIN_12,
		GPIO_PIN_13,
		TIM2,
		TIM_CHANNEL_1,
		168,
		PWM,
		PWM_RES
	}
};
