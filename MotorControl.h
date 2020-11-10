/*
 * MotorControl.h
 *
 *  Created on: Sep 3, 2020
 *      Author: vanti
 */
#pragma once

#include <main.h>


#ifndef SRC_MOTORCONTROL_H_
#define SRC_MOTORCONTROL_H_

typedef struct USR_Encoder_Data
{
	uint8_t EncReso;
	int16_t EncCount;
	uint32_t* EncCurrentVal;
	uint32_t EncPrevVal;
} USR_Encoder_Data;


typedef struct USR_Motor_Data
{
	float RPSec2PWMScale;
	float Velocity2RPSecScale;
	float CurrentRPSec;
	USR_Encoder_Data EncoderData;

} USR_Motor_Data;

void MotorL_EnablePWM(void);
void MotorL_DisablePWM(void);

void MotorR_EnablePWM(void);
void MotorR_DisablePWM(void);

void MotorL_SetPWM(int32_t PWMVal);
void MotorR_SetPWM(int32_t PWMVal);

void MotorR_Brake();
void MotorL_Brake();

void Servo_SetAngle(float ServoAngle);

void Sensor_SetThresHold(uint16_t newThres[]);
#endif /* SRC_MOTORCONTROL_H_ */
