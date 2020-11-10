/*
 * MotorControl.c
 *
 *  Created on: Sep 3, 2020
 *      Author: vanti
 */
#include "MotorControl.h"


void MotorL_EnablePWM(void)
{
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
}

void MotorL_DisablePWM(void)
{
	LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH1);
}


void MotorR_EnablePWM(void)
{
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);
}

void MotorR_DisablePWM(void)
{
	LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
}

void MotorL_SetPWM(int32_t PWMVal) // PWM Val between 0-7200
{
	if(PWMVal >= 7200)
	{
		PWMVal = 7200;
	}
	else if(PWMVal <= -7200)
	{
		PWMVal = -7200;
	}
	if(PWMVal >= 0)
	{
		LL_TIM_OC_SetCompareCH1(TIM1, (uint16_t)PWMVal);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15);
	} else
	{
		LL_TIM_OC_SetCompareCH1(TIM1,7200 + PWMVal);
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15);
	}
}
void MotorR_SetPWM(int32_t PWMVal) // PWM Val between 0-7200
{
	if(PWMVal >= 7200)
	{
		PWMVal = 7200;
	}
	else if(PWMVal <= -7200)
	{
		PWMVal = -7200;
	}
	if(PWMVal >= 0)
	{
		LL_TIM_OC_SetCompareCH3(TIM1, (uint16_t)PWMVal);
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9);
	} else
	{
		LL_TIM_OC_SetCompareCH3(TIM1, 7200 + PWMVal);
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
	}
}
void MotorR_Brake()
{
	LL_TIM_OC_SetCompareCH3(TIM1, 7200);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
}
void MotorL_Brake()
{
	LL_TIM_OC_SetCompareCH1(TIM1, 7200);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9);
}
void Servo_SetAngle(float ServoAngle)
{

	if (ServoAngle > 90) ServoAngle = 90;
	else if (ServoAngle < -90) ServoAngle = -90;
//	uint16_t ServoCPR = 540 + ServoAngle*2;
	LL_TIM_OC_SetCompareCH1(TIM4, 4500 + ServoAngle*50/3);
//	LL_TIM_SetCounter(TIM4, 0);
}

//void Sensor_SetThresHold(uint16_t newThres[])
//{
//	Sensor_Threshold = newThres;
//
//}
