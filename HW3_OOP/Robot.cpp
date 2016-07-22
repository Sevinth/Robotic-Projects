// 
// 
// 

#include "Robot.h"
#include "Motors.h"
#include <LSM303.h>
#include <L3G.h>
#include <Zumo32U4LCD.h>

Zumo32U4Encoders wheelEncoders;
Zumo32U4LCD lcd;

float deltaTheta;
float thetaAvg;

void RobotClass::init()
{

	lastTime = 0.0;
	deltaTime = 0.0;

	encoderLeftCounts = 0.0;
	encoderRightCounts = 0.0;

	//Initiate wheel velocity variables
	rightWheelVel = 0.0f;
	leftWheelVel = 0.0f;

	//Initial distance variables
	totalRightWheelDistance = 0.0f;
	totalLeftWheelDistance = 0.0f;
	totalDistance = 0.0f;

	//Initiate poses to 0 since robot starting is assumed to be <0,0>
	currentRPos.x = 0.0f;
	currentRPos.y = 0.0f;
	currentRPos.theta = 0.0f;


	previousRPos.x = 0.0f;
	previousRPos.y = 0.0f;
	previousRPos.theta = 0.0f;


	globalCurrentPos.x = 0.0f;
	globalCurrentPos.y = 0.0f;
	globalCurrentPos.theta = 0.0f;


	globalPreviousPos.x = 0.0f;
	globalPreviousPos.y = 0.0f;
	globalPreviousPos.theta = 0.0f;


	//Initate the IMU structs
	currGyro.xRot = 0;
	currGyro.yRot = 0;
	currGyro.zRot = 0;

	prevGyro.xRot = 0;
	prevGyro.yRot = 0;
	prevGyro.zRot = 0;


	currAcc.xAcc = 0;
	currAcc.yAcc = 0;
	currAcc.zAcc = 0;

	prevAcc.xAcc = 0;
	prevAcc.yAcc = 0;
	prevAcc.zAcc = 0;

	encoderLeftCounts = 0;
	encoderRightCounts = 0;

	encoderDeltaLeft = 0;
	encoderDeltaRight = 0;
	
	//Initiate quadrature encoders and IMU to default parameters
	//Gyro Default: +/- 250 mpds, conversion: 8.75 mpds/LSB
	//Accel Default: +/- 2 g, conversion: 0.61 mg/LSB

	Wire.begin();
	lcd.init();

	wheelEncoders.init();
	rGyro.init();
	rGyro.enableDefault();
	rAccel.init();
	rAccel.enableDefault();

	gyroCalibration();
	accCalibration();

	wheelEncoders.getCountsAndResetLeft();
	wheelEncoders.getCountsAndResetRight();
	
}

void RobotClass::calcRDistances() {

	encoderLeftCounts = wheelEncoders.getCountsAndResetLeft();
	encoderRightCounts = wheelEncoders.getCountsAndResetRight();

	if (isnan(encoderLeftCounts)) encoderLeftCounts = 0;
	if (isnan(encoderRightCounts)) encoderRightCounts = 0;

	//First, determine distance traveled via the encoder sensors
	 leftDistance = encoderLeftCounts*rWheelCirc / encoderRes;
	 rightDistance = encoderRightCounts*rWheelCirc / encoderRes;
	 centerDistance = 0.5f*(rightDistance + leftDistance);

}

void RobotClass::updatePosition(RobotClass::sensorsAllowed sensors) {
	
	deltaTime = millis() - lastTime;

	if (sensors = RobotClass::sensorsAllowed::IMU_ENC) {
		
		//Calculate distances based on encoders
		calcRDistances();

		//Calculate Velocities based on encoders
		calcRVelocities(leftDistance, rightDistance, deltaTime);

		//Update total distance counters
		updateDistances(leftDistance, rightDistance, centerDistance);
		
		//Second, read values from the IMU
		readIMU();
		
		//Calculate change in orientation using IMU
		deltaTheta = (prevGyro.zRot + currGyro.zRot)*deltaTime/2000.0f;
		
		//testing out the complimentary filter

		//Update the robots pose
		updateRobotPose(centerDistance, leftDistance, rightDistance, deltaTheta, deltaTime);
		updateGlobalPose(centerDistance, leftDistance, rightDistance);
	
		lcd.gotoXY(0, 0);
		lcd.print("X:");
		lcd.gotoXY(2, 0);
		lcd.print(globalCurrentPos.x);

		lcd.gotoXY(0, 1);
		lcd.print("Y:");
		lcd.gotoXY(2, 1);
		lcd.print(globalCurrentPos.y);
	}


	lastTime = millis();
}


void RobotClass::calcTurnVel(float turnRadius) {

	float angular = 10;  //"desired speed"

	turnVel.right = angular*(1 - rWheelBase/(2.0f*turnRadius));
	turnVel.leftVel = angular*(1 + rWheelBase / (2.0f*turnRadius));
	
}



//Keep track of the total distance traveled by each wheel and the robots center
void RobotClass::updateDistances(float &leftDist, float &rightDist, float &totalDist) {

	this->totalLeftWheelDistance += leftDist;
	this->totalRightWheelDistance += rightDist;
	this->totalDistance += totalDist;

}



//Update robots position in the local coordinate frame
void RobotClass::updateRobotPose(float &centerDist, float &leftDist, float &rightDist, float &dTheta, unsigned long &dTime) {
	

	float deltaThetaEncoder = (rightDist - leftDist) / rWheelBase;

	float thetaDiff = deltaThetaEncoder - (dTheta);
	
	thetaAvg = 0.5f*(deltaThetaEncoder + (dTheta));
	
	currentRPos.x = previousRPos.x + centerDist;
	currentRPos.y = 0.0f;
	currentRPos.theta = previousRPos.theta + compFilter(dTime, dTheta, deltaThetaEncoder);



	//Constrain anglular position to be between 0 and 2pi
	if (currentRPos.theta > 2.0f * M_PI) {
		currentRPos.theta -= 2.0f * M_PI;
	}
	else if (currentRPos.theta < 0.0f) {
		currentRPos.theta += 2.0f*M_PI;
	}
	
	
	//Save for use in the next loop
	previousRPos.x = currentRPos.x;
	previousRPos.y = currentRPos.y;
	previousRPos.theta = currentRPos.theta;

}


void RobotClass::updateGlobalPose(float &centerDist, float &leftDist, float &rightDist) {

	//Update robots position in the global coordinate frame
	globalCurrentPos.x = globalPreviousPos.x + centerDist*cosf(currentRPos.theta);
	globalCurrentPos.y = globalPreviousPos.y + centerDist*sinf(currentRPos.theta);
	globalCurrentPos.theta = currentRPos.theta;

	//Save for use in the next loop
	globalPreviousPos.x = globalCurrentPos.x;
	globalPreviousPos.y = globalCurrentPos.y;
	globalPreviousPos.theta = globalCurrentPos.theta;
}


void RobotClass::readIMU() {
	
	rGyro.read();
	
	
	
	if (rGyro.g.z >= gyroNoise || rGyro.g.z <= -gyroNoise) {
	
		currGyro.xRot = rGyro.g.x;
		currGyro.xRot -= gyroOffsets.xRot;

		currGyro.yRot = rGyro.g.y;
		currGyro.yRot -= gyroOffsets.yRot;

		prevGyro.zRot = currGyro.zRot;

		currGyro.zRot = rGyro.g.z;
		currGyro.zRot -= gyroOffsets.zRot;
	}
	else {
		currGyro.zRot = 0;
	}
	
	rAccel.read();

	if (rAccel.a.x >= 2.0*fabs(accXNoise)|| rAccel.a.x<= - 2.0*fabs(accXNoise)) {

		currAcc.xAcc = rAccel.a.x - accOffsets.xAcc;
	}
	else {
		currAcc.xAcc = 0;
	}


	if (rAccel.a.y >= 2.0*fabs(accYNoise) || rAccel.a.y <= -2.0*fabs(accYNoise)) {

		currAcc.yAcc = rAccel.a.y - accOffsets.yAcc;

	}
	else {
		currAcc.yAcc = 0;
	}

	if (rAccel.a.z >= accZNoise || rAccel.a.z <= -accZNoise) {
		currAcc.zAcc = rAccel.a.z - accOffsets.zAcc;
	}

	convAccVals();
	convGyroVals();	


}

void RobotClass::convGyroVals() {

	currGyro.xRot *= GYRO_CONV;
	currGyro.yRot *= GYRO_CONV;
	currGyro.zRot *= GYRO_CONV;

}

void RobotClass::convAccVals() {

	currAcc.xAcc *= ACC_CONV;
	currAcc.yAcc *= ACC_CONV;
	currAcc.zAcc *= ACC_CONV;

}


void RobotClass::gyroCalibration() {

	gyroNoise = 0;
	
	int nSamples = 500;

	for (int i = 0; i < nSamples; i++) {
		rGyro.read();
		currGyro.xRot += rGyro.g.x;
		currGyro.yRot += rGyro.g.y;
		currGyro.zRot += rGyro.g.z;
	}

	gyroOffsets.xRot = currGyro.xRot / nSamples;
	gyroOffsets.yRot = currGyro.yRot / nSamples;
	gyroOffsets.zRot = currGyro.zRot / nSamples;

	
	//Code modified version of code available from 
	// L3GD20 3 Axis Gyro Lab
	//Website: http://web.csulb.edu/~hill/ee444/Labs/5%20Gyro/5%20Arduino%20IDE%20Gyro.pdf
	for (int i = 0; i < nSamples; i++) {
		rGyro.read();
		if (rGyro.g.z - gyroOffsets.zRot > gyroNoise) gyroNoise = rGyro.g.z - gyroOffsets.zRot;
		else if (rGyro.g.z - gyroOffsets.zRot < -gyroNoise) gyroNoise = -rGyro.g.z - gyroOffsets.zRot;
	}

	gyroNoise /= nSamples;

	currGyro.xRot = 0.0;
	currGyro.yRot = 0.0;
	currGyro.zRot = 0.0;


}

void RobotClass::accCalibration() {

	int nSamples = 500;

	for (int i = 0; i < nSamples; i++) {
		rAccel.read();
		currAcc.xAcc += rAccel.a.x;
		currAcc.yAcc += rAccel.a.y;
		currAcc.zAcc += rAccel.a.z;
	}

	accOffsets.xAcc = currAcc.xAcc / nSamples;
	
	accOffsets.yAcc = currAcc.yAcc / nSamples;

	accOffsets.zAcc = currAcc.zAcc / nSamples;

	for (int i = 0; i < nSamples; i++) {
		rGyro.read();
		if (rAccel.a.x - accOffsets.xAcc > accXNoise) accXNoise = rAccel.a.x - accOffsets.xAcc;
		else if (rAccel.a.x - accOffsets.xAcc < -accXNoise) accXNoise = -rAccel.a.x - accOffsets.xAcc;
	}

	for (int i = 0; i < nSamples; i++) {
		rGyro.read();
		if (rAccel.a.y - accOffsets.yAcc > accYNoise) accYNoise = rAccel.a.y - accOffsets.yAcc;
		else if (rAccel.a.y - accOffsets.yAcc < -accYNoise) accYNoise = -rAccel.a.y - accOffsets.yAcc;
	}
}

float RobotClass::compFilter(unsigned long deltaTime, float sensor1, float sensor2) {

	float accAngle;
	if (currAcc.yAcc > 0.05 || currAcc.xAcc > 0.05) {
		accAngle = atan2(currAcc.yAcc, currAcc.xAcc);
	}
	else {
		accAngle = 0;
	}

	float angVal = COMP_F*(sensor1)+(1.0f - COMP_F)*accAngle;


	angVal = COMP_F_B*(angVal)+(1.0f - COMP_F_B)*sensor2;

	//Filtered angular position,
	return angVal;

}


int RobotClass::getRightCounts() {
	
	return encoderRightCounts;

}

int RobotClass::getLeftCounts() {

	return encoderLeftCounts;
}

void RobotClass::rotateInPlace(float angle) {


	if (angle > 2 * M_PI) {
		angle -= 2 * M_PI;
	}
	else if (angle < 0) {
		angle += 2 * M_PI;
	}

	float tempAng = globalCurrentPos.theta;

	while (globalCurrentPos.theta > angle + 0.08 || globalCurrentPos.theta < angle - 0.08 ) {

		motors.setRightMotorSpeed(-75);
		motors.setLeftMotorSpeed(75);
		updatePosition(RobotClass::IMU_ENC);
	}

	//Stop robot turning once it gets to the correct angle
	robotStop();


}
void RobotClass::moveTo(RobotClass::rPosition waypoint, RobotClass::pathType path) {

	if (path == RobotClass::DIRECT) {
	
		//Determine where the robot is
		updatePosition(RobotClass::IMU_ENC);

		//Distance and direction to objective
		float distVec[2] = { 0,0 };
		float distMag;

		//Distance vector to waypoint

			distVec[0] = waypoint.x - globalCurrentPos.x;
			distVec[1] = waypoint.y - globalCurrentPos.y;

		//Magnitude of distance vector
		for (int i = 0; i < 2; i++) {
			distMag += pow(distVec[i], 2);
		}
		distMag = sqrt(distMag);

		//Angle to face directly at waypoint
		float angleToWP;
		float alphaAngle;

		alphaAngle = atan2(distVec[1], distVec[2]);
		angleToWP = globalCurrentPos.theta + alphaAngle;
		//Turn to face the waypoint
		rotateInPlace(angleToWP);

		leftWheelVel = 75;
		rightWheelVel = 75;

		while (distMag >= 1.75) {

			motors.setLeftMotorSpeed(leftWheelVel);
			motors.setRightMotorSpeed(leftWheelVel);
			updatePosition(RobotClass::IMU_ENC);

			if (encoderLeftCounts < encoderRightCounts) {
				leftWheelVel++;
				rightWheelVel--;
			}
			else if (encoderLeftCounts > encoderRightCounts) {
				rightWheelVel++;
				leftWheelVel--;
			}

			distVec[0] = waypoint.x - globalCurrentPos.x;
			distVec[1] = waypoint.y - globalCurrentPos.y;
			
			for (int i = 0; i < 2; i++) {
				distMag += pow(distVec[i], 2);
			}
			distMag = sqrt(distMag);


			
		}



	}

	else if (path == RobotClass::CURVED) {
		//Maintain the proper turn radius
		calcTurnVel(11.0f);

		leftMotorSpeed = 1.2*turnVel.leftVel;
		rightMotorSpeed = 0.9*turnVel.right;

		float velRatio = leftMotorSpeed / rightMotorSpeed;

		float currentTurnRadius;

		while (globalCurrentPos.theta >= M_PI + 0.1 || globalCurrentPos.theta <= M_PI) {


			motors.setLeftMotorSpeed(leftMotorSpeed * 5);
			motors.setRightMotorSpeed(rightMotorSpeed * 5);

			updatePosition(RobotClass::IMU_ENC);
			lcd.gotoXY(0, 0);
			lcd.print(globalCurrentPos.theta);

			float centerDist = (0.5f*rWheelCirc / encoderRes)*(encoderLeftCounts + encoderRightCounts);

			currentTurnRadius = fabs((centerDist / thetaAvg));

			lcd.gotoXY(0, 1);
			lcd.print(currentTurnRadius);

			if (currentTurnRadius > 11.0f) {
				leftMotorSpeed += 1;
				rightMotorSpeed -= 1;
			}
			else if (currentTurnRadius < 11.0f) {
				leftMotorSpeed -= 1;
				rightMotorSpeed += 1;
			}


			if (globalCurrentPos.theta <= M_PI + 0.5 && globalCurrentPos.theta >= M_PI - 0.5) exit;



		}

		motors.setRightMotorSpeed(0);
		motors.setLeftMotorSpeed(0);

	}


}

//Used to set new waypoints from the main program(the ino file)
void RobotClass::setWaypoint(float x, float y, float theta, waypointNames name) {

	switch (name) {
	case 0:
		rWaypoints.wpone.x = x;
		rWaypoints.wpone.y = y;
		rWaypoints.wpone.theta = theta;
		break;
	case 1:
		rWaypoints.wptwo.x = x;
		rWaypoints.wptwo.y = y;
		rWaypoints.wptwo.theta = theta;
		break;
	}


}


void RobotClass::calcRVelocities(float &lDist, float &rDist, unsigned long &dTime) {

	rVel.rightWheel = 1000.0f*rDist/ dTime;
	rVel.leftWheel = 1000.0f*lDist / dTime;
	rVel.center = 0.5*(rVel.rightWheel + rVel.leftWheel);
	rVel.angular = (1.0 / rWheelBase)*(rVel.rightWheel - rVel.leftWheel);

}


//Set velocity of the left wheel in inches/second
void RobotClass::setLeftWheelVelocity(float vel) {


	while (rVel.leftWheel > vel + 0.2 || rVel.leftWheel < vel - 0.2) {
		
		updatePosition(RobotClass::IMU_ENC);
		if (vel - rVel.leftWheel > 0) {

			leftWheelVel++;
			motors.setLeftMotorSpeed(leftWheelVel);

		}
		else if (vel - rVel.leftWheel < 0) {
			
			leftWheelVel--;
			motors.setLeftMotorSpeed(leftWheelVel);
			
		}
		delay(100);
	}

}

//Set Velocity of the right wheel in inches/second
void RobotClass::setRightWheelVelocity(float vel) {
	rightWheelVel = 50;
	leftWheelVel = 50;

	unsigned long last;

	while (rVel.rightWheel > vel + 0.5 || rVel.leftWheel < vel - 0.5) {
		unsigned long dT = millis() - last;

		updatePosition(RobotClass::IMU_ENC);
		if (vel - rVel.rightWheel > 0) {

			rightWheelVel++;
			motors.setRightMotorSpeed(rightWheelVel);
		}
		else if (vel - rVel.rightWheel < 0) {
			rightWheelVel--;
			motors.setRightMotorSpeed(rightWheelVel);

		}

		last = millis();
		delay(100);
	}


}


RobotClass Robot;

