
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



RobotClass robot;

RobotClass::sensorsAllowed sensors = RobotClass::IMU_ENC;
RobotClass::pathType path1 = RobotClass::DIRECT;
RobotClass::pathType path2 = RobotClass::CURVED;
RobotClass::pathType path3 = RobotClass::DIRECT;
RobotClass::pathType path4 = RobotClass::DIRECT;

void setup()
{
	Serial.begin(115200);

	robot.init();
	


	robot.motors.setLeftMotorDirection(true);
	robot.motors.setRightMotorDirection(true);
	robot.setWaypoint(22.0f, 0.0f, 0.0f, RobotClass::WaypointOne);
	robot.setWaypoint(22.0f, -22.0f, M_PI, RobotClass::WaypointTwo);
	robot.setWaypoint(0.0f, -22.0f, M_PI, RobotClass::WaypointThree);
	robot.setWaypoint(0.0f, 0.0f, M_PI / 2.0f, RobotClass::WaypointThree);
}

void loop()
{
	



	//robot.rotateInPlace(M_PI / 2.0);

	//robot.moveTo(robot.getWaypointOne(), path1);
	//robot.robotStop();
	delay(100);
	robot.moveTo(robot.getWaypointTwo(), path1);
	delay(100);
	robot.robotStop();
	robot.moveTo(robot.getWaypointThree(), path3);
	delay(100);
	robot.moveTo(robot.getWaypointFour(), path4);

	

}
