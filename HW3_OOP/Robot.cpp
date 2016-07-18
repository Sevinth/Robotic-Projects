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
		deltaTheta = currGyro.zRot*deltaTime/1000.0f;
		
		//testing out the complimentary filter
		float test = compFilter(deltaTime);

		//Update the robots pose
		updateRobotPose(centerDistance, leftDistance, rightDistance, deltaTheta);
		updateGlobalPose(centerDistance, leftDistance, rightDistance);
	
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
void RobotClass::updateRobotPose(float &centerDist, float &leftDist, float &rightDist, float &dTheta) {
	

	float deltaThetaEncoder = (rightDist - leftDist) / rWheelBase;

	float thetaDiff = deltaThetaEncoder - (dTheta);
	
	thetaAvg = 0.5f*(deltaThetaEncoder + (dTheta));
	
	currentRPos.x = previousRPos.x + centerDist;
	currentRPos.y = 0.0f;
	currentRPos.theta = previousRPos.theta + (dTheta);



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
	
	
	//if (rGyro.g.z >= gyroNoise || rGyro.g.z <= -gyroNoise) {
	
		currGyro.xRot = rGyro.g.x;
		currGyro.xRot -= gyroOffsets.xRot*GYRO_X_OFFSET_GAIN;

		currGyro.yRot = rGyro.g.y;
		currGyro.yRot -= gyroOffsets.yRot*GYRO_Y_OFFSET_GAIN;

		currGyro.zRot = rGyro.g.z;
		currGyro.zRot -= gyroOffsets.zRot;
	//}
	
	rAccel.read();

	currAcc.xAcc = rAccel.a.x - accOffsets.xAcc;

	currAcc.yAcc = rAccel.a.y - accOffsets.yAcc;
	
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

	

	/*for (int i = 0; i < nSamples; i++) {
		rGyro.read();
		if (rGyro.g.z - gyroOffsets.zRot > gyroNoise) gyroNoise = rGyro.g.z - gyroOffsets.zRot;
		else if (rGyro.g.z - gyroOffsets.zRot < -gyroNoise) gyroNoise = -rGyro.g.z - gyroOffsets.zRot;
	}*/   

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
}

float RobotClass::compFilter(unsigned long deltaTime) {

	float accAngle = atan2(currAcc.yAcc, currAcc.xAcc);

	float angVal = COMP_F*(currentRPos.theta + currGyro.zRot*(deltaTime/1000.0f)) + (1.0f - COMP_F)*accAngle;

	//Filtered angular position, works poorly
	return angVal;

}


int RobotClass::getRightCounts() {
	
	return encoderRightCounts;

}

int RobotClass::getLeftCounts() {

	return encoderLeftCounts;
}


void RobotClass::moveTo(RobotClass::rPosition waypoint, RobotClass::pathType path) {

	if (path == RobotClass::DIRECT) {
		//Make sure the robot drives in a straight line

		leftMotorSpeed = 75;
		rightMotorSpeed = 75;
		float distVec[2] = { 0,0 };

		float dist = 10;
		float global;
		
		
		//Update the robots position in global and local coordinate frames
		
		while (abs(dist) > 4.0 && abs(dist) < 24.0) {
			

			motors.setLeftMotorSpeed(leftMotorSpeed);
			motors.setRightMotorSpeed(rightMotorSpeed);
			updatePosition(RobotClass::IMU_ENC);
			
			delay(20);
			float deltaEncoderCounts = encoderRightCounts - encoderLeftCounts;

			distVec[0] = globalCurrentPos.x - waypoint.x;
			distVec[1] = globalCurrentPos.y - waypoint.y;

			for (int i = 0; i < 2; i++) {
				dist += pow(distVec[i],2);
			}
			dist = sqrt(dist);

			if (globalCurrentPos.theta > waypoint.theta + 0.2) {

				leftMotorSpeed += 2;
				rightMotorSpeed -= 2;

			}
			else if (globalCurrentPos.theta < waypoint.theta - 0.2) {
				leftMotorSpeed -= 2;
				rightMotorSpeed += 2;
			}

			
			//Adjust motor PWM based on the difference in encoder counts every loop
			if (deltaEncoderCounts > 0 ) {
				leftMotorSpeed += 2;
				rightMotorSpeed -= 2;
			}
			else if (deltaEncoderCounts < 0 ) {
				leftMotorSpeed -= 2;
				rightMotorSpeed += 2;
			}
		} 

		motors.setLeftMotorSpeed(0);
		motors.setRightMotorSpeed(0);
	}
	else if (path == RobotClass::CURVED) {
		//Maintain the proper turn radius
		 calcTurnVel(11.0f);
		 
		 leftMotorSpeed = 1.2 *turnVel.leftVel;
		 rightMotorSpeed = 0.9*turnVel.right;
		 
		

		 
  			 
		 float velRatio =	 leftMotorSpeed / rightMotorSpeed;

		 float currentTurnRadius;
		 
			  while (globalCurrentPos.theta >= M_PI + 0.1 || globalCurrentPos.theta <= M_PI - 0.1) {
				

				  encoderLeftCounts = wheelEncoders.getCountsAndResetLeft();
				  encoderRightCounts = wheelEncoders.getCountsAndResetRight();

				  motors.setLeftMotorSpeed(leftMotorSpeed * 8);
				  motors.setRightMotorSpeed(rightMotorSpeed * 8);

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

