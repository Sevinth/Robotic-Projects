#include <Wire.h>
#include <Zumo32U4\Zumo32U4.h>
#include <Zumo32U4\L3G.h>
#include <Zumo32U4\LSM303.h>
#include <math.h>
#include <float.h>
#include <stdarg.h>

#define COMP_F_A 0.98f
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

int prevCountsRight  = 0;
int prevCountsLeft = 0;

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
float gPath[5][3] = { { 22,0, 0 },
{ 0,-22.0f,pi},
{ 0,0,0 },
{ 0,0,0 },
{ 0,0,0 } };
float posTol[3] = { 0.5, 0.5, 0.1 }; //inches
Path rPath;
Path::pathType rPathType;

//Time
unsigned long timeSinceLast = 0.0;
unsigned long prevTime = 0.0;

//motors
float leftMotorSpeed = 100;
float rightMotorSpeed = 100;
float deltaEncodersLeft;
float deltaEncodersRight;

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
float deltaEncoderLeft = 0;
float deltaEncoderRight = 0;

bool usingIMUOnly;

void getRVel(float deltaCountsRight, float deltaCountsLeft, float timeEllapsed) {

<<<<<<< HEAD
	//Use difference in counts for Left and Right motors to calculate the forwad velocities
	float rVelRight = deltaCountsRight*wheelCirc/(timeEllapsed*encoderRes);
	float rVelLeft = deltaCountsLeft*wheelCirc / (timeEllapsed*encoderRes);

	//Center velocity of the robot
	float rVelCenter = 0.5f*(rVelRight + rVelLeft);
	//Angular velocity of the robot
	float rVelAng = (1.0f / baseWidth)*(rVelRight - rVelLeft);
=======
	float rDeltaCenter = 0.5*(deltaCountsRight + deltaCountsLeft);

	float rVelCenter = rDeltaCenter*wheelCirc / (timeEllapsed*encoderRes);
	float rVelRight = deltaCountsRight*wheelCirc / (timeEllapsed*encoderRes);
	float rVelLeft = deltaCountsLeft*wheelCirc / (timeEllapsed*encoderRes);
	float rVelAng = (1.0 / baseWidth)*(rVelRight - rVelLeft);
>>>>>>> origin/master

	//Copy velocities into global variable for use later
	rVelocities[0] = rVelCenter;
	rVelocities[1] = rVelRight;
	rVelocities[2] = rVelLeft;
	rVelocities[3] = rVelAng;


}


//Not currently used
float straightLineCalc(float distance) {

	float time;

	time = distance / rVelocities[0];

	return time;

}


//Get angular velocity around the Z axis according to the gyro
float getZRot(float deltaTime) {

	//Read the gyro
	float nSamples = 15;
	float gyroZRotRaw = 0;

	for (int i = 0; i < nSamples; i++) {
		gyro.read();
		
		gyroZRotRaw += gyro.g.z;
	}
	
	gyroZRotRaw /= nSamples;
	
	float gyroZRot = gyroZRotRaw/8.75; // millidegrees / seceond

	gyroZRot = (pi / (180.0f*1000.0f))*gyroZRot;
	gyroZRot = gyroZRot;

	return gyroZRot*deltaTime;

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

float calcTurnRatios(float turnRadius) {

	float velLeft;
	float velRight;

	velLeft =  200.0f* (1 + baseWidth / (2.0*turnRadius));
	velRight = 200.0f* (1 - baseWidth / (2.0*turnRadius));


	return velLeft / velRight;
}


//Called durinv movment to update the robots pose
void updatePosition() {

	float timeBegin = millis();
	timeSinceLast = millis() - prevTime;
	
	//Get right and left encoder Counts
	float encoderLeft = encoders.getCountsLeft();
	float encoderRight = encoders.getCountsRight();
	//Get difference between previous and current encoder counts
	deltaEncoderLeft = encoderLeft - prevCountsLeft;
	deltaEncoderRight = encoderRight - prevCountsRight;

	getRVel(deltaEncoderRight, deltaEncoderLeft, timeSinceLast);

	float deltaRight = wheelCirc*deltaEncoderRight / encoderRes;
	float deltaLeft = wheelCirc*deltaEncoderLeft / encoderRes;
	float deltaCenter = 0.5f*(deltaRight + deltaLeft);

	float deltaTheta = (1.0f / baseWidth)*(deltaRight - deltaLeft);



<<<<<<< HEAD
	//Robot Coordinate Frame Pose
	rCurrentPose[0] = rPrevPose[0] + rVelocities[0] * timeSinceLast;
	rCurrentPose[1] = 0.0f;
	rCurrentPose[2] = rPrevPose[2] + rVelocities[3] * timeSinceLast;
	
	if (rCurrentPose[2] < 0.0f) {
		rCurrentPose[2] = 2.0f * pi;
	}
	else if (rCurrentPose[2] > 2.0f * pi) {
		rCurrentPose[2] = 0.0f;
	}
=======
>>>>>>> origin/master
	

	//Save current counts for use in next updatePosition()
	prevCountsRight = encoderRight;

	prevCountsLeft = encoderLeft;

	//Get velocities from delta encoder values
	//getRVel(deltaEncoderRight, deltaEncoderLeft, timeSinceLast);


	if (usingIMUOnly) {
	
		float deltaThetaIMU = angCompFilter(timeSinceLast);

		//Robot Coordinate Frame Pose
		rCurrentPose[0] = rPrevPose[0] + deltaCenter;
		rCurrentPose[1] = 0.0f;
		//rCurrentPose[2] = angCompFilter(timeSinceLast);
		rCurrentPose[2] = rPrevPose[2] + deltaThetaIMU;

		//Constrain angle(rCurrentPose[2]) between 0 and 2*pi
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


		//Update robot pose in the global coordinate frame
		gCurrentPose[0] = gPrevPose[0] + deltaCenter*cos(rCurrentPose[2]);
		gCurrentPose[1] = gPrevPose[1] + deltaCenter*sin(rCurrentPose[2]);
		gCurrentPose[2] = rCurrentPose[2];

	}
	else {

		float filteredAngle;

		//Robot Coordinate Frame Pose
		rCurrentPose[0] = rPrevPose[0] + deltaCenter;
		rCurrentPose[1] = 0.0f;
		//rCurrentPose[2] = angCompFilter(timeSinceLast);
		rCurrentPose[2] = filteredAngle;

		//Constrain angle(rCurrentPose[2]) between 0 and 2*pi
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


		//Update robot pose in the global coordinate frame
		gCurrentPose[0] = gPrevPose[0] + deltaCenter*cos(rCurrentPose[2]);
		gCurrentPose[1] = gPrevPose[1] + deltaCenter*sin(rCurrentPose[2]);
		gCurrentPose[2] = rCurrentPose[2];

	}


	prevTime = millis();


	//Save current poses for use in the next updatePosition()
	for (int i = 0; i < 3; i++) {
		gPrevPose[i] = gCurrentPose[i];
		rPrevPose[i] = rCurrentPose[i];
	}

	float timeEnd = millis() - timeBegin;

}
//Complimentary filter
float angCompFilter(float timeEllapsed) {

	//Gyro rotation about Z axis
	float zGyro = getZRot(timeEllapsed); //Angular displacement about the Z axis according to the gyro
	getAccValues();



	float accTheta = atan2f(accYVals[0], accXVals[0]);

	//Apply complimentary filter to gyrosocope and accelerometer values
	float imuAngVal = COMP_F_A*(rPrevPose[2] + zGyro) + (1.0f - COMP_F_A)*(accTheta);

	return imuAngVal;

}

void gyroCalibrate() {

	//Make sure there is no rotation
	motors.setSpeeds(0, 0);
	//Read the gyro
	float ellapsedTime = 0;
	float averageZeroReading = 0;
	int numberofReadings = 250;

	for (int i = 0; i < numberofReadings; i++) {

		//while (!gyro.readReg(L3G::STATUS_REG) & 0x08);
		gyro.read();
		delay(10);
		averageZeroReading += gyro.g.z;
		Serial.println("Calibration Gyro Read:");
		Serial.print(averageZeroReading);
		Serial.println("Run #:");
		Serial.print(numberofReadings);
		Serial.println("");
	}

	gyroZOffest = averageZeroReading / (float)numberofReadings;
	Serial.println("Gyro Offset");
	Serial.print(gyroZOffest);

}


//Not used Currently
void courseCorrection(int waypoint) {

	//determine distance to next waypoint
	float distVec[3] = { 0,0,0 };
	float yErr;
	float xErr;

	int turnDir = 0;
	

	
	for (int i = 0; i < 2; i++) {
		distVec[i] = gPath[waypoint][i] - gCurrentPose[i];
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
		angleToWaypoint = atan2(distVec[1], distVec[0]);

		if (angleToWaypoint < 0.0f) {

			angleToWaypoint *= -1;
			turnDir = -1; //Left Turn
		} else if (angleToWaypoint > 0.0f) {
		
			turnDir = 1;
		}
	}

	if (rPathType == Path::DIRECT) {
		float encoderDeltas = deltaEncoderRight - deltaEncoderLeft;
		
		if(encoderDeltas > 0.0f) {

			leftMotorSpeed += 5;
			rightMotorSpeed -= 5;
		}
		else if (encoderDeltas < 0.0f) {
		
			leftMotorSpeed -= 5;
			rightMotorSpeed += 5;
		}

		//if (abs(angleToWaypoint) > 1.0) {
		//	motors.setSpeeds(0, 0);
		//	rotateInPlace(angleToWaypoint, turnDir);
		//	delay(10);
		//}


	}
	else if (rPathType == Path::CURVE) {

		float velRatio = calcTurnRatios(11.0f);

		float denom = fabs(deltaEncoderLeft - deltaEncoderRight);
		float CalculatedRadius = (baseWidth / 2.0f)*(deltaEncoderRight + deltaEncoderLeft) / (denom);

		if (CalculatedRadius > 11.0f) {
			leftMotorSpeed += 5;
			rightMotorSpeed -= 5;

		}
		else if (CalculatedRadius < 11.0f) {
			leftMotorSpeed -= 5;
			rightMotorSpeed += 5;
		}
			
		leftMotorSpeed = constrain(leftMotorSpeed, 75, 225);
		rightMotorSpeed = constrain(rightMotorSpeed, 75, 225);

	}
	


}

//Not used currently
void rotateInPlace(float angle, int turnDirection) {

	float leftSpeed = 0;
	float rightSpeed = 0;

	if (turnDirection < 0) {
		leftSpeed = -200;
		rightSpeed = 200;

		while (rCurrentPose[2] <= angle) {
			motors.setSpeeds(leftSpeed, rightSpeed);
			updatePosition();
			
		}
	}
	else if (turnDirection > 0) {
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

	leftMotorSpeed = 100;
	rightMotorSpeed = 100;
	float xTol, yTol;
	xTol = 0.5;
	yTol = 0.5;

	if (rPathType == Path::DIRECT) {

		while ((gCurrentPose[0] <= gPath[waypoint][0] - 0.5 || gCurrentPose[0] >= gPath[waypoint][0] + 0.5)) {
			motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
			updatePosition();
			courseCorrection(waypoint);

		}


		motors.setSpeeds(0, 0);
	}
	else if (rPathType == Path::CURVE) {

		rightMotorSpeed = 100;
		leftMotorSpeed = calcTurnRatios(11.0f)*rightMotorSpeed*1.5;
		motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
		delay(250);
		while ((gCurrentPose[2]  >= gPath[waypoint][2]) || (gCurrentPose[2] <= gPath[waypoint][2])) {
			motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
			updatePosition();
			lcd.gotoXY(0, 0);
			lcd.print(gCurrentPose[2]);
			lcd.gotoXY(0, 1);

			courseCorrection(waypoint);

		}

		motors.setSpeeds(0, 0);


	}



}

//Simple accelerometer calibration
void accCalibrate() {
	
	lcd.gotoXY(0, 0);
	lcd.print("Calibrate");
	lcd.gotoXY(0, 1);
	lcd.print("Accel");


	int nSamples = 1024;
	float accReadX  = 0.0f;
	float accReadY = 0.0f;
	float accReadZ = 0.0f;
	

<<<<<<< HEAD
=======
	for (int i = 0; i < nSamples; i++) {
		acc.read();
		delay(10);
		accReadX += acc.a.x;
		accReadY += acc.a.y;
		accReadZ += acc.a.z;
		
	}
>>>>>>> origin/master

	accReadX /= (float)nSamples;
	accReadY /= (float)nSamples;
	accReadZ /= (float)nSamples;

	accXOffset =  accReadX;
	accYOffset =  accReadY;
	accZOffset =  accReadZ; 

}


//Retrive accelerometer values and convert to mpds
void getAccValues() {

	acc.read();
	int nSamples = 15;

	for (int i = 0; i < nSamples; i++) {
		accXVals[0] += ((float)acc.a.x + accXOffset)*0.61f;

		accYVals[0] += ((float)acc.a.y + accYOffset)*0.61f;

		accZVals[0] += ((float)acc.a.z + accZOffset)*0.61f;
	}

<<<<<<< HEAD
		accXVals[0] = ((float)acc.a.x - accXOffset)*0.61f;

		accYVals[0] = ((float)acc.a.y - accYOffset)*0.61f;

		accZVals[0] = ((float)acc.a.z)*0.61f;
=======
	accXVals[0] /= nSamples;
	accYVals[0] /= nSamples;
	accZVals[0] /= nSamples;


>>>>>>> origin/master
	
}

void setup()
{
	//Serial.begin(9600);
	//while (!Serial) {}
	//enable communication to I2C devices
	Wire.begin();

<<<<<<< HEAD
	
	//Initiate L3G with default behavior, then calibrate
=======
	//Initiate encoders
>>>>>>> origin/master
	encoders.init();
	//Initiate gyr and enable default values (+/- 250 dps)
	gyro.init();

	gyro.enableDefault();
<<<<<<< HEAD
	gyroCalibrate();

	//Initiate LSM303 with default behavior, then calibrate
=======
	gyro.writeReg(L3G::CTRL1, 0b11111111);
	gyroCalibrate();

	//Enable accelerometer with default values (+/- 2g)
>>>>>>> origin/master
	acc.init();
	acc.enableDefault();
	//accCalibrate();

	//Reset encoder counts to zero
	encoders.getCountsAndResetLeft();
	encoders.getCountsAndResetRight();

	//Motors were wired in backwards by someone
	motors.flipLeftMotor(true);
	motors.flipRightMotor(true);

}

void loop()
{

	/* add main program code here */
	float volts = readBatteryMillivolts();
	bool rightError = encoders.checkErrorRight();
	bool leftError = encoders.checkErrorLeft();
	rPathType = Path::CURVE;

	motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);

	//Go to First Waypoint
	usingIMUOnly = true;
	rPathType = Path::DIRECT;
	moveToLocation(0);

	//Go to Second Waypoint
	rPathType = Path::CURVE;
	moveToLocation(1);


	while (true) {
		motors.setSpeeds(0, 0);
	}


}