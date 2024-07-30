#pragma once
#include "main.h"
#include "math.h"
#define MIN(a,b) ((a)<(b)?(a):(b)) //takes param "A" & "B" if A is less than B then A if not then B 
#define MAX(a,b) ((a)>(b)?(a):(b)) //takes param "A" & "B" if A is greather than than B then A if not then B 

#define AUTO_NUMBER 12 //can'tused static types for array len or const uint soo womp cope 
extern uint8_t auton; 

extern pros::Controller controller;

//Declare motors
extern pros::Motor frontright;
extern pros::Motor midright;
extern pros::Motor backright;

extern pros::Motor frontleft;
extern pros::Motor midleft;
extern pros::Motor backleft;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor lift;
extern pros::Motor intake;

//Declare V5 sensors
extern pros::Imu imu;
extern pros::Rotation horizontalTracker;
extern pros::Rotation verticalTracker;
extern pros::Rotation liftRot;

/*ADI Digital out*/
extern pros::adi::DigitalOut mogoMechPisses;

extern bool backClampTog;