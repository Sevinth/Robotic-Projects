#include <Wire.h>
#include <Zumo32U4\Zumo32U4.h>
#include <Zumo32U4\L3G.h>
#include <Zumo32U4\LSM303.h>
#include <math.h>
#include <float.h>

#define COMP_F_A 0.3f
#define COMP_F_B 0.5f

Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;
Zumo32U4Motors motors;
L3G gyro;
LSM303 acc;

//Path type
struct Path {
	enum pathType {DIRECT, CURVE};
	float curveRadius;
	float pathLength;
};

//Physical Constants
const float pi = 4.0f * atanf(1.0f);

//Robot Physical Properties
const float wheelRad = 0.691f;
const float baseWidth = 3.351f;
const float wheelCirc = 2.0f * pi*wheelRad;;
const float encoderRes = 75.81f*12.0f;

//Position Variables
float rCurrentPose[3] = { 0,0,0 };
float rPrevPose[3] = { 0,0,0 };
float rVelocities[4] = { 0,0,0,0 };
float gCurrentPose[3] = { 0,0,0 };
float gPrevPose[3] = { 0,0,0 };
float gVelocities[4] = { 0,0,0,0 };
float gPath[5][3] = { { 0,0, pi },
{ 0,0,0 },
{ 0,0,0 },
{ 0,0,0 },
{ 0,0,0 } };
const float posTol = 0.5; //inches
Path rPath;
Path::pathType rPathType;

//Time
unsigned long timeSinceLast = 0.0;
unsigned long prevTime = 0.0;

//motors
float leftMotorSpeed = 0;
float rightMotorSpeed = 0;

//calibration variables
float gyroZOffest = 0;
float accXOffset = 0;
float accYOffset = 0;
float accZOffset = 0;
int accXVals[2] = { 0,0 };
int accYVals[2] = { 0,0 };
int accZVals[2] = { 0,0 };


//Testing
float encoderCounts = 0;

void getRVel(float deltaCountsRight, float deltaCountsLeft, float timeEllapsed) {

	
	float rVelRight = deltaCountsRight*wheelCirc/(timeEllapsed*encoderRes);
	float rVelLeft = deltaCountsLeft*wheelCirc / (timeEllapsed*encoderRes);

	float rVelCenter = 0.5f*(rVelRight + rVelLeft);
	float rVelAng = (1.0f / baseWidth)*(rVelRight - rVelLeft);

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
	float gyroZRot = (pi/180.0*1000.0)*gyroZRotRaw*(1.0 / 8.75);

	return gyroZRot;

}


//Fourth Order Runge-Kutta
float *rk4(float timeEllapsed) {

	float k1[4] = { 0,0,0,0 };
	float k2[4] = { 0,0,0,0 };
	float k3[4] = { 0,0,0,0 };

	float posVec[3] = { 0,0,0 };

	k1[0] = rVelocities[0] * cos(gPrevPose[2]);
	k2[0] = rVelocities[0] * sin(gPrevPose[2]);
	k3[0] = rVelocities[3];
	
	k1[1] = rVelocities[0] * cos(gPrevPose[2] + k3[0] * (timeEllapsed / 2.0f));
	k2[1] = rVelocities[0] * sin(gPrevPose[2] + k3[0] * (timeEllapsed / 2.0f));
	k3[1] = k3[0];

	k1[2] = rVelocities[0] * cos(gPrevPose[2] + k3[1] * (timeEllapsed / 2.0f));
	k2[2] = rVelocities[0] * sin(gPrevPose[2] + k3[1] * (timeEllapsed / 2.0f));
	k3[2] = k3[1];

	k1[3] = rVelocities[0] * cos(gPrevPose[2] + k3[2] * timeEllapsed);
	k2[3] = rVelocities[0] * sin(gPrevPose[2] + k3[2] * timeEllapsed);
	k3[3] = k3[2];

	posVec[0] = (1.0f / 6.0f)*(k1[0] + 2.0f*(k1[1] + k1[2]) + k1[3]);
	posVec[1] = (1.0f / 6.0f)*(k2[0] + 2.0f*(k2[1] + k2[2]) + k2[3]);
	posVec[2] = (1.0f / 6.0f)*(k3[0] + 2.0f*(k3[1] + k3[2]) + k3[3]);

	return posVec;
}


//Called durinv movment to update the robots pose
void updatePosition() {

	float timeBegin = millis();
	timeSinceLast = millis() - prevTime;

	float encoderLeft = encoders.getCountsAndResetLeft();
	float encoderRight = encoders.getCountsAndResetRight();

	getRVel(encoderRight, encoderLeft, timeSinceLast);

	float gyroAngVel = getZRot(timeSinceLast);

	//rVelocities[3] = (rVelocities[3] + gyroAngVel) / 2.0f;

	//Robot Coordinate Frame Pose
	rCurrentPose[0] = rPrevPose[0] + rVelocities[0] * timeSinceLast;
	rCurrentPose[1] = 0.0f;
	rCurrentPose[2] = angCompFilter(timeSinceLast);
	
	if (rCurrentPose[2] < 0.0f) {
		rCurrentPose[2] = 2.0f * pi;
	}
	else if (rCurrentPose[2] > 2.0f * pi) {
		rCurrentPose[2] = 0.0f;
	}
	
	//Global Coordinate Frame Pose
	
	//float rk4Time = millis();
	//float *deltaPosEst = rk4(timeSinceLast);
	//float rk4TimeEnd = millis() - rk4Time;

	gCurrentPose[0] = gPrevPose[0] + rVelocities[0] * timeSinceLast*cos(rCurrentPose[2]);
	gCurrentPose[1] = gPrevPose[1] + rVelocities[0] * timeSinceLast*sin(rCurrentPose[2]);
	gCurrentPose[2] = rCurrentPose[2];

	if (gCurrentPose[2] < 0.0f) {
		gCurrentPose[2] = 2.0f * pi;
	}
	else if (gCurrentPose[2] > 2.0f * pi) {
		gCurrentPose[2] = 0.0f;
	}

	delay(20);
	prevTime = millis();

	for (int i = 0; i < 3; i++) {
		gPrevPose[i] = gCurrentPose[i];
		rPrevPose[i] = rCurrentPose[i];
	}

	float timeEnd = millis() - timeBegin;

}
//Complimentary filter
float angCompFilter(float timeEllapsed) {

	//Need accelerometer values
	getAccValues();

	//Calculate the angle between Y and X accel values
	float rotAcc = atan2(accYVals[0], accXVals[0]);
	//Gyro rotation about Z axis
	float zGyro = getZRot(timeEllapsed);
	//Apply computational filter to gyrosocope and accelerometer values
	float imuAngVal = COMP_F_A*(rPrevPose[2] + zGyro*timeEllapsed) + (1.0f - COMP_F_A)*(rotAcc);

	return imuAngVal;

}

void gyroCalibrate() {

	//Make sure there is no rotation
	motors.setSpeeds(0, 0);
	//Read the gyro
	float ellapsedTime = 0;
	float averageZeroReading = 0;
	int numberofReadings = 1024;

	for (int i = 0; i < numberofReadings; i++) {

		//while (!gyro.readReg(L3G::STATUS_REG) & 0x08);
		gyro.read();

		numberofReadings++;

		gyro.read();
		averageZeroReading += gyro.g.z;

	}

	gyroZOffest = averageZeroReading / (float)numberofReadings;

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

	float angleToWaypoint;

	if (distVec[0] == 0 && distVec[1] == 0)  {

		angleToWaypoint = rCurrentPose[2] + gPath[waypoint][2];
	}
	else if (distVec[0] != 0 || distVec[0] != 0) {
		angleToWaypoint = atan2(distVec[0], distVec[1]);
	}

	if (rPathType == Path::DIRECT) {
		rotateInPlace(angleToWaypoint);
		moveToLocation(waypoint);


	} else if (rPathType == Path::CURVE)  {
	
	}
	


}

void rotateInPlace(float angle) {

	float leftSpeed = 0;
	float rightSpeed = 0;

	if (angle > 0) {
		leftSpeed = -200;
		rightSpeed = 200;

		while (rCurrentPose[2] <= angle) {
			motors.setSpeeds(leftSpeed, rightSpeed);
			updatePosition();
		}
	}
	else if (angle < 0) {
		leftSpeed = 200;
		rightSpeed = -200;

		while (rCurrentPose[2] >= angle) {
			motors.setSpeeds(leftSpeed, rightSpeed);
			updatePosition();
		}


	}

	motors.setSpeeds(0, 0);

}

void moveToLocation(int waypoint) {


}

void accCalibrate() {
	

	int nSamples = 1024;
	float accReadX  = 0.0f;
	float accReadY = 0.0f;
	float accReadZ = 0.0f;

	for (int i = 0; i < nSamples; i++) {
		acc.read();
		delay(1);
		accReadX += acc.a.x;
		accReadY += acc.a.y;
		accReadZ += acc.a.z;
		
	}

	accReadX /= (float)nSamples;
	accReadY /= (float)nSamples;
	accReadZ /= (float)nSamples;

	accXOffset =  accReadX;
	accYOffset =  accReadY;
	accZOffset =  accReadZ; 

}

void getAccValues() {

	acc.read();
	int nSamples;

		accXVals[0] = ((float)acc.a.x + accXOffset)*0.61f;

		accYVals[0] = ((float)acc.a.y + accYOffset)*0.61f;

		accZVals[0] = ((float)acc.a.z + accZOffset)*0.61f;
	
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
	//gyroCalibrate();

	acc.init();
	acc.enableDefault();
	accCalibrate();

	encoders.getCountsAndResetLeft();
	encoders.getCountsAndResetRight();
	motors.flipLeftMotor(true);
	motors.flipRightMotor(true);

}

void loop()
{

	/* add main program code here */
	float volts = readBatteryMillivolts();
	bool rightError = encoders.checkErrorRight();
	bool leftError = encoders.checkErrorLeft();
	int move = 0;

	motors.setSpeeds(100, 100);
	updatePosition();



}