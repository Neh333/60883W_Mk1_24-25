#include "drive.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "util.hpp"
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
slewProfile mogoProfile{90, 30, 70};

void winPoint(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.setScheduledConstants(PIDConstants[4]);

 drive.addErrorFunc(10, LAMBDA(drive.setMaxVelocity(50)));
 drive.move(backward, 34, 2, 100);

 mogoMechPisses.set_value(true);

 intake.move_voltage(12000);
 pros::delay(700);

 drive.setPID(4);
 drive.setScheduleThreshold_a(20);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.setSlew(mogoProfile);

 drive.turn(left, imuTarget(260), 2, 70);
 
 drive.addErrorFunc(6, LAMBDA(drive.setMaxVelocity(60)));
 drive.move(forward, 26, 2, 100);
  
 drive.turn(shortest, 60, 2, 70);

 pros::delay(700);

 mogoMechPisses.set_value(false);
 
 drive.setPID(8);
 drive.setScheduleThresholds_s(0, 0);
 drive.setScheduledSwerveConstants(PIDConstants[6]);

 drive.swerve(forwardRight, 50, imuTarget(90), 3, 100, 20);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void leftSide(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightSide(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void leftElim(){
  pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightElim(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void skills(){
  pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void nothing(){}

void tune(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 drive.setScheduleThreshold_l(10);

 
 drive.setScheduleThreshold_a(20);
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);

 drive.setSlew(mogoProfile);

 /*
 drive.move(forward, 22, 1, 100);
 pros::delay(1000);
 drive.move(forward, 32, 2, 100);
 pros::delay(1000);
 drive.move(forward, 42, 2, 100);
 pros::delay(2000);
 */ 

 
 drive.turn(right, 50, 1, 70);
 pros::delay(1000);

 drive.turn(right, 60, 1, 70);
 pros::delay(1000);

 drive.turn(right, 75, 1, 70);
 pros::delay(1000);

 drive.turn(right, 80, 1, 70);
 pros::delay(1000);

 drive.turn(right, 100, 1, 70);
 pros::delay(1000);

 drive.turn(right, 120, 2, 70);
 pros::delay(1000);
 
 drive.turn(right, 135, 2, 70);
 pros::delay(1000);
 

 drive.turn(right, 150, 2, 70);
 pros::delay(1000);

 drive.turn(right, 175, 2, 70);
 pros::delay(1000);

 drive.turn(right, 180, 2, 70);
 pros::delay(1000);

 drive.turn(right, 195, 2, 70);
 pros::delay(1000);

 drive.turn(right, 205, 2, 70);
 pros::delay(1000);
 

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}