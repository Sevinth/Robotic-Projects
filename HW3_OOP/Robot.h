// Robot.h

#ifndef _ROBOT_h
#define _ROBOT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"

#else
	#include "WProgram.h"
#endif


#include <Wire.h>
#include "Motors.h"
#include <L3G.h>
#include <LSM303.h>
#include <Zumo32U4Encoders.h>


#define GYRO_CONV (8.75/1000.0f) //* (M_PI/180.0f) //Convert Gyro output to radians/second
#define GYRO_X_OFFSET_GAIN 1.0
#define GYRO_Y_OFFSET_GAIN 1.0
#define GYRO_Z_OFFSET_GAIN 2.0


#define ACC_CONV 0.061f/1000.0f
#define ACC_X_OFFSET_GAIN 1.0
#define ACC_Y_OFFSET_GAIN 1.0
#define ACC_Z_OFFSET_GAIN 1.0

#define COMP_F 0.98f
#define GYRO_OFFEST 0.0015f

class RobotClass
{
protected:

	//Physical parameters of the robot
	const float rWheelBase = 3.351; //Wheel base in inches
	const float rWheelRad = 0.691;  //Wheel radius in inches
	const float encoderRes = 75.81*12.0; //Encoder resolution
	const float rWheelCirc = 2 * M_PI*rWheelRad; //Wheel circumference in inches

	const float goalRadius = 1; // +/- 1 inch of goal
	//Conversion factors for gyro and accelerometer

	float totalDistance;
	float totalLeftWheelDistance;
	float totalRightWheelDistance;
	
	int rightMotorSpeed;
	int leftMotorSpeed;

	float rightWheelVel;  // Right wheel velocity
	float leftWheelVel;  // Left Wheel velocity

	//Store encoder values for left and right wheels
	int encoderLeftCounts;
	int encoderRightCounts;

	int encoderDeltaLeft;
	int encoderDeltaRight;

	float leftDistance;
	float rightDistance;
	float centerDistance;

	float gyroNoise;

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


	struct rVelocities {
		float leftWheel;
		float rightWheel;
		float center;
		float angular;
	};
	struct turnVelocities {
		float leftVel;
		float right;
	};

	

	struct wayPoints {
		rPosition wpone;
		rPosition wptwo;
		rPosition wpthree;
		rPosition wpfour;
	};



	turnVelocities turnVel;

	wayPoints rWaypoints;

	rotVel currGyro;
	rotVel prevGyro;

	rotVel gyroOffsets;
	

	linAcc currAcc;
	linAcc prevAcc;

	linAcc accOffsets;

	//Create instances of the rPosition struct
	//for all both reference frames
	rPosition currentRPos;
	rPosition previousRPos;
	rPosition globalCurrentPos;
	rPosition globalPreviousPos;

	rVelocities rVel;

	L3G rGyro;
	LSM303 rAccel;
	

public:

	Motors motors;

	unsigned long deltaTime;
	unsigned long lastTime;

	enum sensorsAllowed { IMU, IMU_ENC };
	enum pathType {DIRECT, CURVED};
	enum waypointNames { WaypointOne, WaypointTwo, WaypointThree, WaypointFour };

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


	void calcRDistances();

	void calcTurnVel(float);


	void moveTo(rPosition, pathType);

	void setWaypoint(float x, float y, float theta, waypointNames name);

	//Get functions for all the structs
	rPosition getCurrentRPos() {
		return this->currentRPos;
	}

	rPosition getCurrentGPos() {
		return this->globalCurrentPos;
	}

	rPosition getWaypointOne() {
		return this->rWaypoints.wpone;
	}

	rPosition getWaypointTwo() {
		return this->rWaypoints.wptwo;
	}

	rPosition getWaypointThree() {
		return this->rWaypoints.wpthree;
	}

	rPosition getWaypointFour() {
		return this->rWaypoints.wpfour;
	}

	

	void calcRVelocities(float &leftDist, float &rightDist, unsigned long &dT);
	
	void setLeftWheelVelocity(float);
	void setRightWheelVelocity(float);


	void gyroCalibration();
	void accCalibration();
};





extern RobotClass Robot;

#endif

