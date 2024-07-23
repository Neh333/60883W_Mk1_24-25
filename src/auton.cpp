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

                           /*{kP, kPa, kI, kIa,  kD,  kDa,  kPd}*/
PIDprofile weirdMogoConstants{0,  130,  0,   0,   0,  200,      0};

                                /*{kP, kPa, kI, kIa,  kD,  kDa,  kPd}*/
PIDprofile weirdMogoTurnsScheduled{0,  55,  0,  15,   0,  700,  0};

void winPoint(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.setScheduledConstants(PIDConstants[4]);

 drive.addErrorFunc(10, LAMBDA(drive.setMaxVelocity(40)));
 drive.move(backward, 34, 2, 100);

 mogoMechPisses.set_value(true);

 pros::delay(200);

 intake.move_voltage(12000);

 pros::delay(500);

 drive.setCustomPID(weirdMogoConstants);
 drive.setScheduledConstants(weirdMogoTurnsScheduled);
 drive.setScheduleThreshold_a(20);
 drive.setSlew(mogoProfile);
                           
 /*IMPORTANT: two turns need tuned ig cause like weirrd friction */

 
 drive.turn(left, imuTarget(260), 2, 70);

 pros::delay(1300); //discard afetr testing
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.addErrorFunc(6, LAMBDA(drive.setMaxVelocity(60)));
 drive.move(forward, 26, 2, 100);

 drive.setScheduledConstants(weirdMogoTurnsScheduled);
 drive.turn(shortest, 50, 2, 70);

 pros::delay(1500); //keep this delay even afetr testing at 1500 msec

 /*
 mogoMechPisses.set_value(false);
 
 drive.setPID(8);
 drive.setScheduleThresholds_s(0, 10);
 drive.setScheduledSwerveConstants(PIDConstants[6]);
 
 //170 degree angular movememt
 drive.swerve(forwardRight, 52, imuTarget(85), 3, 60, 5);

 pros::delay(1500); //to check error 
 
 drive.setPID(1);
 drive.setScheduledConstants(PIDConstants[4]);
 drive.move(forward, 24, 2, 60);
 
 drive.addErrorFunc(50, LAMBDA(intake.move_voltage(0)));
 drive.turn(right, imuTarget(155), 1, 70);

 pros::Task liftUp{[]{ 
   while(liftRot.get_position()>120) 
   {
      lift.move_voltage(11000);
   }
  }};

 drive.setPID(2);
 drive.setScheduleThreshold_l(NO_SCHEDULING);
 drive.move(backward, 16, 1, 100);

 intake.move_voltage(12000);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();*/
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
 /*
 drive.setScheduleThreshold_l(10);
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.setScheduleThreshold_a(20);

 drive.setSlew(mogoProfile);
 */
 drive.setPID(8);
 drive.setScheduleThresholds_s(0, 10);
 drive.setScheduledSwerveConstants(PIDConstants[6]);
 
 //170 degree angular movememt
 drive.swerve(forwardRight, 52, 40, 3, 60, 10);
 pros::delay(1000);

 /*
 drive.move(forward, 22, 1, 100);
 pros::delay(1000);
 drive.move(forward, 32, 2, 100);
 pros::delay(1000);
 drive.move(forward, 42, 2, 100);
 pros::delay(2000);
 */ 
 /*x
 
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
   */

 

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}