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
	delay(1);
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

	float desiredV = 10;  //"desired speed"

	turnVel.right = desiredV*(1.0 - rWheelBase/(2.0f*turnRadius));
	turnVel.leftVel = desiredV*(1.0 + rWheelBase / (2.0f*turnRadius));
	
}



//Keep track of the total distance traveled by each wheel and the robots center
void RobotClass::updateDistances(float &leftDist, float &rightDist, float &totalDist) {

	this->totalLeftWheelDistance += leftDist;
	this->totalRightWheelDistance += rightDist;
	this->totalDistance += totalDist;

}



//Update robots position in the local coordinate frame
void RobotClass::updateRobotPose(float &centerDist, float &leftDist, float &rightDist, float &dTheta, unsigned long &dTime) {
	

	//Save for use in the next loop
	previousRPos.x = currentRPos.x;
	previousRPos.y = currentRPos.y;
	previousRPos.theta = currentRPos.theta;

	float deltaThetaEncoder = (rightDist - leftDist) / rWheelBase;

	//Not used
	float thetaDiff = deltaThetaEncoder - (dTheta);
	
	//Not used
	thetaAvg = 0.5f*(deltaThetaEncoder + (dTheta));
	

	// update robot pose
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
	


}


void RobotClass::updateGlobalPose(float &centerDist, float &leftDist, float &rightDist) {
	//Save for use in the next loop
	globalPreviousPos.x = globalCurrentPos.x;
	globalPreviousPos.y = globalCurrentPos.y;
	globalPreviousPos.theta = globalCurrentPos.theta;


	//Update robots position in the global coordinate frame
	globalCurrentPos.x = globalPreviousPos.x + centerDist*cosf(currentRPos.theta);
	globalCurrentPos.y = globalPreviousPos.y + centerDist*sinf(currentRPos.theta);
	globalCurrentPos.theta = currentRPos.theta;


}


void RobotClass::readIMU() {
	
	rGyro.read();
	
	
	//If values read from gyro are not larger than the noise value
	//determined during calibration, sets rotation value to 0
	//Acceleration noise behaves the same way
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


	//Convert raw values to something more useful
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

	//Average over the sample range
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


	//Average over the sample range
	accOffsets.xAcc = currAcc.xAcc / nSamples;
	
	accOffsets.yAcc = currAcc.yAcc / nSamples;

	accOffsets.zAcc = currAcc.zAcc / nSamples;


	//Code modified version of code available from:
	// L3GD20 3 Axis Gyro La
	//Website: http://web.csulb.edu/~hill/ee444/Labs/5%20Gyro/5%20Arduino%20IDE%20Gyro.pdf
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


/*
Computational filter used to combine acc + gyro data, then (acc+gyro) + encoder data
in order to provide a delta angle estimation
*/
float RobotClass::compFilter(unsigned long deltaTime, float sensor1, float sensor2) {

	float accAngle;
	float yAcc;
	float xAcc;
	if (fabs(currAcc.yAcc) > 0.03){
		
		yAcc = currAcc.yAcc;
	}
	else {
		
		yAcc = 0.0;
	}

	if (fabs(currAcc.xAcc) > 0.03) {
		xAcc = currAcc.xAcc;
	}
	else {
		xAcc = 0.0;
	}

	accAngle = atan2(yAcc, xAcc);
	
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

//Zero point turn
void RobotClass::rotateInPlace(float angle) {

	lcd.clear();
	float startAngle = globalCurrentPos.theta;
	float temp;
	//Positive angle = left turn
	if (angle > 0) {
		motors.setRightMotorSpeed(50);
		motors.setLeftMotorSpeed(-50);
	}
	//negative angle = right turn
	else if (angle < 0) {
		motors.setRightMotorSpeed(-50);
		motors.setLeftMotorSpeed(50);
	}

	//Check angle rotated agains magnitude of angle argument
	while (fabs(globalCurrentPos.theta - startAngle) < fabs(angle) - 0.08 || fabs(globalCurrentPos.theta - startAngle) > fabs(angle) + 0.08) {
		updatePosition(RobotClass::IMU_ENC);
	}

	//Stop robot turning once it gets to the correct angle
	robotStop();
	

}
void RobotClass::moveTo(RobotClass::rPosition waypoint, RobotClass::pathType path) {


	//If moving in a straight line, path will be DIRECT
	if (path == RobotClass::DIRECT) {
	
		int heading;

		//Give each motor an equal speed
		motors.setLeftMotorSpeed(75);
		motors.setRightMotorSpeed(75);

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

		//Not doing anything with the heading variable right now
		if (abs(distVec[0]) > abs(distVec[1])) heading = 0;
		else heading = 1;

		//Angle to face directly at waypoint
		float angleToWP;
		float alphaAngle;
		
		//Calculate angle to the waypoint from current position
		alphaAngle = atan2(distVec[1], distVec[0]);
		//Determine how far the robot should rotate
		angleToWP = globalCurrentPos.theta - alphaAngle;

		/*
		If the current angle is larger than PI a positive angle
		results from the atan2 function

		However in a situation where the robot has an orientation of
		theta = 3*PI/2 and the alphaAngle is PI, the robot should make a right zero
		point turn, but it will make a left since the resultant angle is positive
		So we multiply it by a negative 1
		*/
		if (globalCurrentPos.theta > M_PI) angleToWP = -angleToWP;

		//Turn to face the waypoint
		rotateInPlace(angleToWP);

		
		leftWheelVel = 75;
		rightWheelVel = 75;

		float prevDistMag;
		//Get within some distance of the target waypoint
		while (distMag >= 2.50) {


			motors.setLeftMotorSpeed(leftWheelVel);
			motors.setRightMotorSpeed(leftWheelVel);


			updatePosition(RobotClass::IMU_ENC);


			//Adjust wheel speeds based on encoder count differences
			//Helps maintain a straight line course
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
			prevDistMag = distMag;
			
			
			for (int i = 0; i < 2; i++) {
				distMag += pow(distVec[i], 2);
			}
			distMag = sqrt(distMag);
			
			//Check the gyro readout and adjust wheel speeds to try and keep a 0 rotation


			if(currGyro.zRot > 0.0) {
				leftWheelVel++;
				rightWheelVel--;
			} else if(currGyro.zRot < 0.0){
				leftWheelVel--;
				rightWheelVel++;
			}
			

	
		}

		//Stop at the waypoint
		robotStop();

	}

	else if (path == RobotClass::CURVED) {
		//Maintain the proper turn radius

		lcd.clear();

		//Calculate the turn velocities for a given turn radius
		calcTurnVel(11.0f);

		rightWheelVel = turnVel.right;
		leftWheelVel = turnVel.leftVel;

		float velRatio = leftWheelVel / rightWheelVel;


	
		rightWheelVel = 50.0;
		leftWheelVel = rightWheelVel*velRatio;
		lcd.clear();

		//Keep turning until robot is facing PI direction
		while (globalCurrentPos.theta >= M_PI + 0.1 || globalCurrentPos.theta <= M_PI) {

			//Set the motor speeds.  The scalar multiples are used
			//since the ration from the calcTurnVel() function
			//doesn't seem to be quite right. 
			//The values sent to setLeftMotorSpeed are not
			//actual wheel velocities, just a PWM value
			motors.setLeftMotorSpeed(1.2*leftWheelVel);
			motors.setRightMotorSpeed(0.9*rightWheelVel);
			//Need to know where we are
			updatePosition(RobotClass::IMU_ENC);

			lcd.gotoXY(0, 0);
			lcd.print(globalCurrentPos.theta);

			float centerDist = (0.5f*rWheelCirc / encoderRes)*(encoderLeftCounts + encoderRightCounts);


		}

		//motors.setRightMotorSpeed(0);
		//motors.setLeftMotorSpeed(0);

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

/*
Calculat Robot velocities, not used for anything currently
*/
void RobotClass::calcRVelocities(float &lDist, float &rDist, unsigned long &dTime) {

	rVel.rightWheel = 1000.0f*rDist/ dTime;
	rVel.leftWheel = 1000.0f*lDist / dTime;
	rVel.center = 0.5*(rVel.rightWheel + rVel.leftWheel);
	rVel.angular = (1.0 / rWheelBase)*(rVel.rightWheel - rVel.leftWheel);

}


//Set velocity of the left wheel in inches/second
//Not currently used
void RobotClass::setLeftWheelVelocity(float vel) {

	if (rVel.leftWheel < vel) {
		leftWheelVel++;
	}
	else if (rVel.rightWheel > 0) {
		leftWheelVel--;
	}


}

//Set Velocity of the right wheel in inches/second
//Not currently used
void RobotClass::setRightWheelVelocity(float vel) {
	
	if (rVel.rightWheel < vel) {
		rightWheelVel++;
	}
	else if (rVel.rightWheel > vel) {
		rightWheelVel--;
	}

}


RobotClass Robot;

