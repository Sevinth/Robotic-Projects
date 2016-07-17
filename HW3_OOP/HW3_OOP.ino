
#include <Zumo32U4ProximitySensors.h>
#include <Zumo32U4Motors.h>
#include <Zumo32U4LineSensors.h>
#include <Zumo32U4LCD.h>
#include <Zumo32U4IRPulses.h>
#include <Zumo32U4Encoders.h>
#include <Zumo32U4Buzzer.h>
#include <Zumo32U4Buttons.h>
#include <Zumo32U4.h>
#include <USBPause.h>
#include <QTRSensors.h>
#include <Pushbutton.h>
#include <PololuHD44780.h>
#include <PololuBuzzer.h>
#include <LSM303.h>
#include <L3G.h>
#include <FastGPIO.h>
#include <Wire.h>

#include "Robot.h"
#include "Motors.h"

Motors motors;
RobotClass robot;

RobotClass::sensorsAllowed sensors = RobotClass::IMU_ENC;
RobotClass::pathType path = RobotClass::DIRECT;
void setup()
{
	Serial.begin(9600);
	motors.setRightMotorDirection(true);
	motors.setLeftMotorDirection(true);

	robot.init();


}

void loop()
{

	
	while (robot.getCurrentGPos().x <= 22.0) {
		motors.setRightMotorSpeed(50);
		motors.setLeftMotorSpeed(50);
		robot.updatePosition(sensors);
	}


	motors.setRightMotorSpeed(0);
	motors.setLeftMotorSpeed(0);



	delay(150);

}
