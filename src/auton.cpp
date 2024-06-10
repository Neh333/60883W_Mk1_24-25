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

//Drive drive(leftMotors, rightMotors, imu); // none odom to tune and relative movements






void winPoint(){

 
}























void leftSide(){
 drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightSide(){
 drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void leftElim(){
 drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightElim(){
 drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void skills(){
 drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void nothing(){}

void tune(){
 drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}