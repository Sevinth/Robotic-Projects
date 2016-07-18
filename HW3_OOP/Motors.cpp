#include "Motors.h"


static bool leftMotorReverse = false;
static bool rightMotorReverse = false;

void Motors::setLeftMotorSpeed(int16_t pwmVal) {


	bool reverse = 0;

	//Check PWM value
	if (pwmVal < -400 || pwmVal > 400) {
		pwmVal = constrain(pwmVal, -400, 400);
	}

	this->lMotorPWM = pwmVal;

	if (pwmVal < 0) {
		pwmVal = -pwmVal;
		reverse = 1;
	}
	
	OCR1B = pwmVal;

	FastGPIO::Pin<LEFT_MOTOR_DIRECTION_PIN>::setOutput(reverse ^ leftMotorReverse);
	
}

void Motors::setRightMotorSpeed(int16_t pwmVal) {

	bool reverse = 0;

	//Check PWM value
	if (pwmVal < -400 || pwmVal > 400) {
		pwmVal = constrain(pwmVal, -400, 400);
	}
		
	this->rMotorPWM = pwmVal;

	if (pwmVal < 0) {
		pwmVal = -pwmVal;
		reverse = 1;
	}

	OCR1A = pwmVal;

	FastGPIO::Pin<RIGHT_MOTOR_DIRECTION_PIN>::setOutput(reverse ^ rightMotorReverse);

}

void Motors::setLeftMotorDirection(bool direction) {

	leftMotorReverse = direction;

}

void Motors::setRightMotorDirection(bool direction) {

	rightMotorReverse = direction;

}