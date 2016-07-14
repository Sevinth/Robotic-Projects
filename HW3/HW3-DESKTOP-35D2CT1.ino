

#include <Wire.h>
#include <Zumo32U4\Zumo32U4.h>
#include <Zumo32U4\L3G.h>
#include <Zumo32U4\LSM303.h>
#include <math.h>
#include <float.h>

Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;
Zumo32U4Motors motors;
L3G gyro;
LSM303 acc;

//Physical Constants
const float pi = 4 * atan(1.0);


//Robot Physical Properties
const float wheelRad = 0.691;
const float baseWidth = 3.351;
const float wheelCirc = 2.0 * pi*wheelRad;;
const float encoderRes = 75.81*12.0;

//Position Variables
float rCurrentPose[3] = { 0,0,0 };
float rPrevPose[3] = { 0,0,0 };
float rVelocities[4] = { 0,0,0 ,0 };
float gCurrentPose[3] = { 0,0,0 };
float gPrevPose[3] = { 0,0,0 };
float gVelocities[4] = { 0,0,0 ,0 };
float gPath[5][3] = { { 22,0,0 },
{ 0,0,0 },
{ 0,0,0 },
{ 0,0,0 },
{ 0,0,0 } };


//Time
unsigned long timeSinceLast = 0.0;
unsigned long prevTime = 0.0;

//motors
float leftMotorSpeed = 0;
float rightMotorSpeed = 0;

//calibration variables
float gyroZOffest = 0;

void getRVel(float deltaCountsRight, float deltaCountsLeft, float timeEllapsed) {

	float rDeltaCenter = 0.5*(deltaCountsRight + deltaCountsLeft);

	float rVelCenter = rDeltaCenter*wheelCirc / (timeEllapsed*encoderRes);
	float rVelRight = deltaCountsRight*wheelCirc / (timeEllapsed*encoderRes);
	float rVelLeft = deltaCountsLeft*wheelCirc / (timeEllapsed*encoderRes);
	float rVelAng = (1.0 / baseWidth)*(rVelRight - rVelLeft);

	rVelocities[0] = rVelCenter;
	rVelocities[1] = rVelRight;
	rVelocities[2] = rVelLeft;
	rVelocities[3] = rVelAng;


}

float straightLineCalc(float distance) {

	float time;

	time = distance / rVelocities[0];

	return time;

}

float getZRot(float deltaTime) {

	//Read the gyro
	
	gyro.read();
	
	float gyroZRotRaw = gyro.g.z + gyroZOffest;
	float gyroZRot = (pi/180.0)*gyroZRotRaw*(1.0 / 8.75);

	return gyroZRot;

}

void updatePosition() {

	timeSinceLast = millis() - prevTime;

	float encoderLeft = encoders.getCountsAndResetLeft();
	float encoderRight = encoders.getCountsAndResetRight();

	getRVel(encoderRight, encoderLeft, timeSinceLast);

	float gyroAngVel = getZRot(timeSinceLast);

	float gyroEncoderError = fabs(gyroAngVel - rVelocities[3]);

	//Robot Coordinate Frame Pose
	rCurrentPose[0] = rPrevPose[0] + rVelocities[0] * timeSinceLast;
	rCurrentPose[1] = 0.0;
	rCurrentPose[2] = rPrevPose[2] + rVelocities[3] * timeSinceLast;
	

	if (rCurrentPose[2] < 0) {
		rCurrentPose[2] = 2 * pi;
	}
	else if (rCurrentPose[2] > 2 * pi) {
		rCurrentPose[2] = 0;
	}

	
	//Global Coordinate Frame Pose
	gCurrentPose[0] = gPrevPose[0] + rVelocities[0] * timeSinceLast*cos(rCurrentPose[2]);
	gCurrentPose[1] = gPrevPose[1] + rVelocities[0] * timeSinceLast*sin(rCurrentPose[2]);
	gCurrentPose[2] = rCurrentPose[2];


	delay(20);
	prevTime = millis();

	for (int i = 0; i < 3; i++) {
		gPrevPose[i] = gCurrentPose[i];
		rPrevPose[i] = rCurrentPose[i];
	}

}


void gyroCalibrate() {
	
	//Make sure there is no rotation
	motors.setSpeeds(0, 0);
	//Read the gyro
	gyro.read();


	gyroZOffest = 0.0 - gyro.g.z;


}


void courseCorrection(int waypoint) {

	//determine distance to next waypoint
	float distVec[3] = { 0,0,0 };
	float yErr;
	float xErr;

	for (int i = 0; i < 2; i++) {
		distVec[i] = gCurrentPose[i] - gPath[waypoint][i];
	}

	float dist;

	for (int i = 0; i < 2; i++) {

		dist += sqrt(pow(distVec[i], 2));
	}

	float angleToWaypoint = atan2(distVec[0], distVec[1]);
	float rotAngle = gCurrentPose[2] + angleToWaypoint;
	yErr = distVec[1];
	

}


void moveToLocation(int waypoint) {


	while (gCurrentPose != gPath[waypoint]) {
		updatePosition();
		courseCorrection(1);

	}


}


void setup()
{

	Serial.begin(9600);
	while (!Serial) {}
	//enable communication to I2C devices
	Wire.begin();


	/* add setup code here */
	encoders.init();
	gyro.init();
	gyro.enableDefault();
	gyroCalibrate();

	acc.init();
	acc.enableDefault();

	encoders.getCountsAndResetLeft();
	encoders.getCountsAndResetRight();
	motors.flipLeftMotor(true);
	motors.flipRightMotor(true);

}


void loop()
{

	/* add main program code here */
	
	bool rightError = encoders.checkErrorRight();
	bool leftError = encoders.checkErrorLeft();
	int move = 0;

	updatePosition();
	courseCorrection(0);


	delay(20);

}