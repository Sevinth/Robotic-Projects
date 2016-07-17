// 
// 
// 

#include "Robot.h"

Zumo32U4Encoders wheelEncoders;

void RobotClass::init()
{

	lastTime = 0.0;
	deltaTime = 0.0;


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

	
	//Initiate quadrature encoders and IMU to default parameters
	//Gyro Default: +/- 250 mpds, conversion: 8.75 mpds/LSB
	//Accel Default: +/- 2 g, conversion: 0.61 mg/LSB

	Wire.begin();

	wheelEncoders.init();
	rGyro.init();
	rGyro.enableDefault();
	rAccel.init();
	rAccel.enableDefault();
	
}


void RobotClass::updatePosition(RobotClass::sensorsAllowed sensors) {
	deltaTime = millis() - lastTime;

	if (sensors = RobotClass::sensorsAllowed::IMU_ENC) {
		
		encoderLeftCounts = wheelEncoders.getCountsAndResetLeft();
		encoderRightCounts = wheelEncoders.getCountsAndResetRight();

		if (isnan(encoderLeftCounts)) encoderLeftCounts = 0;
		if (isnan(encoderRightCounts)) encoderRightCounts = 0;

		//First, determine distance traveled via the encoder sensors
		float leftDistance = encoderLeftCounts*rWheelCirc/encoderRes;
		float rightDistance = encoderRightCounts*rWheelCirc / encoderRes;
		float centerDistance = 0.5f*(rightDistance + leftDistance);


		//Update total distance counters
		updateDistances(leftDistance, rightDistance, centerDistance);
		
		//Second, read values from the IMU
		readIMU();
		
		//Calculate change in orientation using IMU
		float deltaTheta = currGyro.zRot*deltaTime/1000.0f;
		
		//testing out the complimentary filter
		float test = compFilter(deltaTime);

		//Update the robots pose
		updateRobotPose(centerDistance, leftDistance, rightDistance, deltaTheta);
		updateGlobalPose(centerDistance, leftDistance, rightDistance);
	
	}




	lastTime = millis();
}


//Keep track of the total distance traveled by each wheel and the robots center
void RobotClass::updateDistances(float &leftDist, float &rightDist, float &totalDist) {

	this->totalLeftWheelDistance += leftDist;
	this->totalRightWheelDistance += rightDist;
	this->totalDistance += totalDist;

}


//Update robots position in the local coordinate frame
void RobotClass::updateRobotPose(float &centerDist, float &leftDist, float &rightDist, float &dTheta) {
	

	float deltaThetaEncoder = (rightDist - leftDist) / rWheelBase;

	float thetaDiff = deltaThetaEncoder - dTheta;
	
	float thetaAvg = 0.5f*(deltaThetaEncoder + dTheta);
	
	currentRPos.x = previousRPos.x + centerDist;
	currentRPos.y = 0.0f;
	currentRPos.theta = previousRPos.theta + dTheta;

	//Constrain anglular position to be between 0 and 2pi
	if (currentRPos.theta > 2.0f*M_PI) {

		currentRPos.theta -= 2.0f*M_PI;

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
	
	rAccel.read();

	currGyro.xRot = rGyro.g.x;
	currGyro.yRot = rGyro.g.y;
	currGyro.zRot = rGyro.g.z;

	currAcc.xAcc = rAccel.a.x;
	currAcc.yAcc = rAccel.a.y;
	currAcc.zAcc = rAccel.a.z;

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


float RobotClass::compFilter(unsigned long deltaTime) {

	float accAngle = atan2(currAcc.yAcc, currAcc.xAcc);

	float angVal = COMP_F*(currentRPos.theta + currGyro.zRot*deltaTime) + (1.0f - COMP_F)*accAngle;

	return angVal;
}


int RobotClass::getRightCounts() {
	
	return encoderRightCounts;

}

int RobotClass::getLeftCounts() {

	return encoderLeftCounts;
}





RobotClass Robot;

