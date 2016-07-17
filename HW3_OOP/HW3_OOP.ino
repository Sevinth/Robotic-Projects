
#include "Robot.h"
#include "Motors.h"

Motors motors;
RobotClass robot;

RobotClass::sensorsAllowed sensors = RobotClass::IMU_ENC;

void setup()
{
	Serial.begin(9600);
	motors.setRightMotorDirection(true);
	motors.setLeftMotorDirection(true);

	robot.init();


}

void loop()
{
	
	//while (robot.getCurrentGPos().theta != 3*M_PI/2.0f) {

	//	motors.setRightMotorSpeed(-100);
	//	motors.setLeftMotorSpeed(100);
	//	robot.updatePosition(sensors);

	//}

	
	motors.setRightMotorSpeed(0);
	motors.setLeftMotorSpeed(0);
	robot.updatePosition(sensors);

	float leftCounts = robot.getLeftCounts();
	float rightCounts = robot.getRightCounts();

	delay(150);

}
