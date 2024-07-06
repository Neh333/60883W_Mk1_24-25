#include "drive.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "util.hpp"
#include <ranges>
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

Drive drive(leftMotors, rightMotors, imu);

void winPoint(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.setScheduledConstants(PIDConstants[3]);
 
 drive.addErrorFunc(8, LAMBDA(intake.move_voltage(12000)));
 drive.move(forward, 36, 2, 90);
 
 
 drive.turn(right, imuTarget(90), 1, 100);

 drive.move(backward, 22, 1, 70);

 mogoMechPisses.set_value(true);

 intake.move_voltage(12000);
 pros::delay(2000);


  
 runOnError.remove();
 drive.onErrorVector.clear();
}

void leftSide(){
 //drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightSide(){
 //drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void leftElim(){
 //drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightElim(){
 //drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void skills(){
 //drive.odom->init();
 pros::Task runOnError(onError_fn);

 runOnError.remove();
 drive.onErrorVector.clear();
}

void nothing(){}

void tune(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.setScheduledConstants(PIDConstants[3]);

 drive.turn(right, 60, 1, 70);
 pros::delay(1000);

 drive.turn(right, 90, 1, 70);
 pros::delay(1000);

 drive.turn(right, 120, 1, 70);
 pros::delay(1000);

 drive.turn(right, 150, 1, 70);
 pros::delay(1000);

 drive.turn(right, 180, 1, 70);
 pros::delay(1000);

 


 runOnError.remove();
 drive.onErrorVector.clear();
}