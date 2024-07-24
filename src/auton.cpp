#include "drive.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "util.hpp"
#include "auton.hpp"
#include <functional>

/* Create an array of auton wrappers  to be used with the auton-selector*/
std::function<void()> autos[AUTO_NUMBER] = {
  {winPointRed},
  {winPointBlue},

  {leftSideRed},
  {leftSideBlue},

  {rightSideRed},
  {rightSideBlue},

  {leftElimRed},
  {leftElimBlue},

  {rightElimRed},
  {rightElimBlue},

  {skills},
  {nothing},
  {tune}
};


Drive drive(leftMotors, rightMotors, imu);
slewProfile mogoProfile{90, 30, 70};

void winPointRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void winPointBlue(){
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

                  /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,  130,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,   55,  0,  15,   0,  750,  0});
 drive.setScheduleThreshold_a(20);
 drive.setSlew(mogoProfile);
                           
 /*IMPORTANT: two turns need tuned ig cause like weirrd friction */

 drive.turn(left, imuTarget(260), 2, 70);

 pros::delay(1000);
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.addErrorFunc(6, LAMBDA(drive.setMaxVelocity(60)));
 drive.move(forward, 26, 2, 100);

                   /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,  420,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,  185,  0,  15,   0,  700,  0});

 drive.turn(shortest, 50, 2, 70);

 pros::delay(1500); //keep this delay even afetr testing at 1500 msec

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
   while(liftRot.get_position()>1200) 
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
 drive.onErrorVector.clear();
}

void leftSideRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}


void leftSideBlue(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightSideRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightSideBlue(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void leftElimRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void leftElimBlue(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}


void rightElimRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void rightElimBlue(){
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