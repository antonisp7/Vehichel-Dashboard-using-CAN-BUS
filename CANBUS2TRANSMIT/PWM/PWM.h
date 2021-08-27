/*
 * File: DC_MOTOR.h
 * Driver Name: [[ DC MOTOR ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Ver: 1.0
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#ifndef PWM_H_
#define PWM_H_

#define HAL_TIM_MODULE_ENABLED

#include "stm32f4xx_hal.h"

// DC Motor Rotation Directions
#define DIR_CW    0
#define DIR_CCW   1

// DC Motor PWM Properties
#define PWM_RES  10
#define PWM    500

// The Number OF DC MOTORs To Be Used In The Project
#define PWM_UNITS  1

typedef struct
{
	GPIO_TypeDef * DIR1_GPIO;
	GPIO_TypeDef * DIR2_GPIO;
	uint16_t       DIR1_PIN;
	uint16_t       DIR2_PIN;
	TIM_TypeDef*   TIM_Instance;
	uint32_t       PWM_TIM_CH;
	uint16_t       TIM_CLK_MHz;
	uint32_t       PWM_FREQ_Hz;
	uint8_t        PWM_RES_BITS;
}PWM_CfgType;


/*-----[ Prototypes For All Functions ]-----*/

void PWM_Init(uint8_t au8_PWM_Instance);
void PWM_Start(uint8_t au8_PWM_Instance, uint8_t au8_DIR, uint16_t au16_SPEED);
void PWM_Set_Speed(uint8_t au8_PWM_Instance, uint16_t au16_SPEED);
void PWM_Set_Dir(uint8_t au8_PWM_Instance, uint8_t au8_DIR);
void PWM_Stop(uint8_t au8_PWM_Instance);
uint32_t PWM_Get_MaxFreq(uint8_t au8_PWM_Instance);


#endif /* PWM_H_ */
