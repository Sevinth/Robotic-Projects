// Robot.h

#ifndef _ROBOT_h
#define _ROBOT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"

#else
	#include "WProgram.h"
#endif

#include "ZumoIncludes.h"
#include <Wire.h>

#define GYRO_CONV (M_PI/180.0f)*8.75/1000.0f
#define ACC_CONV 0.061/1000.0f
#define COMP_F 0.98

class RobotClass
{
protected:

	//Physical parameters of the robot
	const float rWheelBase = 3.351; //Wheel base in inches
	const float rWheelRad = 0.691;  //Wheel radius in inches
	const float encoderRes = 75.81*12.0; //Encoder resolution
	const float rWheelCirc = 2 * M_PI*rWheelRad; //Wheel circumference in inches

	//Conversion factors for gyro and accelerometer

	float totalDistance;
	float totalLeftWheelDistance;
	float totalRightWheelDistance;


	float rightWheelVel;  // Right wheel velocity
	float leftWheelVel;  // Left Wheel velocity

	//Store encoder values for left and right wheels
	int encoderLeftCounts;
	int encoderRightCounts;


	struct rotVel {
		float xRot;
		float yRot;
		float zRot;

	};


	struct linAcc {
		float xAcc;
		float yAcc;
		float zAcc;
	};


	struct rPosition {

		float x;
		float y;
		float theta;

	};


	rotVel currGyro;
	rotVel prevGyro;

	linAcc currAcc;
	linAcc prevAcc;

	//Create instances of the rPosition struct
	//for all both reference frames
	rPosition currentRPos;
	rPosition previousRPos;
	rPosition globalCurrentPos;
	rPosition globalPreviousPos;


	L3G rGyro;
	LSM303 rAccel;

public:

	unsigned long deltaTime;
	unsigned long lastTime;

	enum sensorsAllowed { IMU, IMU_ENC };
	
	
	void init();

	//Update the robots position according to available sensors
	void updatePosition(RobotClass::sensorsAllowed sensors);

	void updateDistances(float &leftDist, float &rightDist, float &totalDist);


	void updateRobotPose(float &centerDist, float &leftDist, float &rightDist, float &dTheta);
	void updateGlobalPose(float &centerDist, float &leftDist, float &rightDist);

	int getLeftCounts();
	int getRightCounts();



	float getTotalDistance() { return this->totalDistance; }
	float getTotalLeftWheelDistance() { return this->totalLeftWheelDistance; }
	float getTotalRightWheelDistance() { return this->totalRightWheelDistance; }


	//IMU Functions
	void readIMU();

	void convGyroVals();
	void convAccVals();

	float compFilter(unsigned long deltaTime);

	//Get functions for all the structs
	rPosition getCurrentRPos() {
		return this->currentRPos;
	}

	rPosition getCurrentGPos() {
		return this->globalCurrentPos;
	}


};




extern RobotClass Robot;

#endif

