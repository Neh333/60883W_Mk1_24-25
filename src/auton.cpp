#include "drive.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "auton.hpp"

/* Create an array of auton-text tuples to be used with the auton-selector */
autonTextTuple autos[AUTO_COUNT] = {
    {winPoint, "Win Point"  },
    {leftSide, "Left Side"  },
    {rightSide,"Right Side" },
    {leftElim, "Left Elims" },
    {rightElim,"Right Elims"},
    {skills,   "Skills"     },
    {nothing,  "Nothing"    },
    {tune,     "Tune"       }
};

//Drive drive(leftMotors, rightMotors, imu); // 
Odometry odom(verticalTracker, horizontalTracker, 2.0, imu);
Drive drive(leftMotors, rightMotors, odom);

void winPoint(){
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void leftSide(){
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightSide(){
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void leftElim(){
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightElim(){
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void skills(){
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void nothing(){}

void tune(){
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}