/*
 * File: DC_MOTOR.c
 * Driver Name: [[ DC MOTOR ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include <PWM_cfg.h>

#include "PWM.h"
#include "main.h"

void PWM_Init(uint8_t au8_PWM_Instance)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_HandleTypeDef htim;
    uint32_t PSC_Value = 0;
    uint32_t ARR_Value = 0;
    uint8_t i = 0;

	/*--------[ Configure The 2 Direction Control GPIO Pins ]-------*/

    if(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO == GPIOA || PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO == GPIOA)
    {
    	__HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO == GPIOB || PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO == GPIOB)
    {
    	__HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO == GPIOC || PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO == GPIOD || PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO == GPIOE || PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
	GPIO_InitStruct.Pin = PWM_CfgParam[au8_PWM_Instance].DIR1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = PWM_CfgParam[au8_PWM_Instance].DIR2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO, &GPIO_InitStruct);
	HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR1_PIN, 0);
	HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR2_PIN, 0);

	/*--------[ Calculate The PSC & ARR Values To Set PWM Resolution And Approx. The F_pwm ]-------*/

	/* Those Equations Sets The PWM Resolution & Approximates The F_pwm */
	ARR_Value = 1;
	for(i=0; i<PWM_CfgParam[au8_PWM_Instance].PWM_RES_BITS; i++)
	{
		ARR_Value *= 2;
	}
	PSC_Value = (uint32_t) ((PWM_CfgParam[au8_PWM_Instance].TIM_CLK_MHz*1000000) / (ARR_Value*PWM_CfgParam[au8_PWM_Instance].PWM_FREQ_Hz));
	PSC_Value--;
	ARR_Value -= 2;

	/*--------[ Configure The DC Motor PWM Timer Channel ]-------*/

	htim.Instance = PWM_CfgParam[au8_PWM_Instance].TIM_Instance;
	htim.Init.Prescaler = PSC_Value;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = ARR_Value;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim);
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig);
	HAL_TIM_PWM_Init(&htim);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH);
	HAL_TIM_MspPostInit(&htim);

	/*--------[ Start The PWM Channel ]-------*/

	HAL_TIM_PWM_Start(&htim, PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH);
}

void PWM_Start(uint8_t au8_PWM_Instance, uint8_t au8_DIR, uint16_t au16_SPEED)
{
	/* Write To The 2 Direction Control Pins */
	if(au8_DIR == DIR_CW)
	{
		HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR1_PIN, 1);
		HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR2_PIN, 0);
	}
	else if(au8_DIR == DIR_CCW)
	{
		HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR1_PIN, 0);
		HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR2_PIN, 1);
	}

	/* Write The Speed Value To The PWM CH DutyCycle Register */
	if(PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH == TIM_CHANNEL_1)
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR1 = au16_SPEED;
	}
	else if(PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH == TIM_CHANNEL_2)
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR2 = au16_SPEED;
	}
	else if(PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH == TIM_CHANNEL_3)
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR3 = au16_SPEED;
	}
	else
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR4 = au16_SPEED;
	}
}

void PWM_Set_Speed(uint8_t au8_PWM_Instance, uint16_t au16_SPEED)
{
	/* Write The Speed Value To The PWM CH DutyCycle Register */
	if(PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH == TIM_CHANNEL_1)
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR1 = au16_SPEED;
	}
	else if(PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH == TIM_CHANNEL_2)
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR2 = au16_SPEED;
	}
	else if(PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH == TIM_CHANNEL_3)
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR3 = au16_SPEED;
	}
	else
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR4 = au16_SPEED;
	}
}

void PWM_Set_Dir(uint8_t au8_PWM_Instance, uint8_t au8_DIR)
{
	/* Write To The 2 Direction Control Pins */
	if(au8_DIR == DIR_CW)
	{
		HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR1_PIN, 1);
		HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR2_PIN, 0);
	}
	else if(au8_DIR == DIR_CCW)
	{
		HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR1_PIN, 0);
		HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR2_PIN, 1);
	}
}

void PWM_Stop(uint8_t au8_PWM_Instance)
{
	/* Write To The 2 Direction Control Pins */
	HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR1_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR1_PIN, 0);
	HAL_GPIO_WritePin(PWM_CfgParam[au8_PWM_Instance].DIR2_GPIO, PWM_CfgParam[au8_PWM_Instance].DIR2_PIN, 0);

	/* Write ZERO To The PWM Ch DutyCycle Register */
	if(PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH == TIM_CHANNEL_1)
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR1 = 0;
	}
	else if(PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH == TIM_CHANNEL_2)
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR2 = 0;
	}
	else if(PWM_CfgParam[au8_PWM_Instance].PWM_TIM_CH == TIM_CHANNEL_3)
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR3 = 0;
	}
	else
	{
		PWM_CfgParam[au8_PWM_Instance].TIM_Instance->CCR4 = 0;
	}
}

uint32_t PWM_Get_MaxFreq(uint8_t au8_PWM_Instance)
{
	uint32_t ARR_Value = 1;
    uint8_t i = 0;

	for(i=0; i<PWM_CfgParam[au8_PWM_Instance].PWM_RES_BITS; i++)
	{
		ARR_Value *= 2;
	}
	return ((PWM_CfgParam[au8_PWM_Instance].TIM_CLK_MHz*1000000)/ARR_Value);
}
