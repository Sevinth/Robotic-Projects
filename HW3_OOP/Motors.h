#pragma once
#define RIGHT_MOTOR_DIRECTION_PIN 15
#define LEFT_MOTOR_DIRECTION_PIN 16
#define RIGHT_MOTOR_SPEED_PIN 9
#define LEFT_MOTOR_SPEED_PIN 10

#include <Arduino.h>
#include <pins_arduino.h>
#include <avr\io.h>
#include <FastGPIO.h>


class Motors {
public:

	Motors() {

		FastGPIO::Pin<RIGHT_MOTOR_DIRECTION_PIN>::setOutputLow();
		FastGPIO::Pin<RIGHT_MOTOR_SPEED_PIN>::setOutputLow();
		FastGPIO::Pin<LEFT_MOTOR_DIRECTION_PIN>::setOutputLow();
		FastGPIO::Pin<LEFT_MOTOR_SPEED_PIN>::setOutputLow();
	
		TCCR1A = 0b10100000;
		TCCR1B = 0b00010001;
		ICR1 = 400;   //Input Compare Register
		OCR1A = 0;    //Output Compare Register 1A
		OCR1B = 0;    //Output Compare Register 1B
	}



private:
	float kpL; // Proportionality Constant for PID Left Motor
	float kdL; // Derivative Constant for PID Left Motor
	float kiL; // Integral Constant for PID Left Motor

	float kpR; //Proportionallity Constant for PID Right Motor
	float kdR; //Derivative Constant for PID Right Motor
	float kiR; //Integral Constant for PID Right Motor

	//PID leftMotorPid;
	//PID rightMotorPid;




public:

	int lMotorPWM;
	int rMotorPWM;


	//Set motor speeds using PWM value
	void setLeftMotorSpeed(int16_t pwmVal);
	void setRightMotorSpeed(int16_t pwmVal);


	// Set the motors direction, true = forward, false = backwards
	void setLeftMotorDirection(bool direction);
	void setRightMotorDirection(bool direction);



};