#include "drive.hpp"
#include "include.hpp"
#include "liblvgl/misc/lv_anim.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "util.hpp"
#include "auton.hpp"
#include "intake.hpp"
#include <functional>

/* Create an array of auton wrappers  to be used with the auton-selector*/
std::function<void()> autos[AUTO_NUMBER] = {
  {winPointRed},
  {winPointBlue},

  {ringSideRed},
  {ringSideBlue},

  {goalSideRed},
  {goalSideBlue},

  {ringElimRed},
  {ringElimBlue},

  {goalElimRed},
  {goalElimBlue},

  {skills},
  {nothing},
  {tune}
};


Drive drive(leftMotors, rightMotors, imu);
slewProfile mogoProfile{90, 30, 70};
IntakeControl conveyor;

void winPointRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void winPointBlue(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.setScheduledConstants(PIDConstants[4]);

 drive.addErrorFunc(12, LAMBDA(drive.setMaxVelocity(25)));
 drive.move(backward, 35, 2, 100);

 mogoMechPisses.set_value(true);

 pros::delay(120); //let the clamp fully actuate

 conveyor.setIntake(400);
 startIntake();

 pros::delay(450); //give the pre load a moment 

                  /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,  130,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,   55,  0,  15,   0,  750,  0});
 drive.setScheduleThreshold_a(20);
 drive.setSlew(mogoProfile);

 drive.turn(left, imuTarget(250), 2, 70);
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 28, 2, 60);

                   /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,   420,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,  190,  0,  15,   0,  700,  0});
 
 drive.turn(shortest, 50, 2, 70);
 pros::delay(300); // give rimg some time

 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]); 

 drive.addErrorFunc(22, LAMBDA(mogoMechPisses.set_value(false)));
 drive.addErrorFunc(20, LAMBDA(drive.setPID(1)));
 drive.addErrorFunc(20, LAMBDA(drive.setScheduledConstants(PIDConstants[4])));
 drive.addErrorFunc(20, LAMBDA( drive.setSlew({0,0,0})));
 drive.move(forward, 45, 2, 100);
 
 drive.setScheduleThreshold_a(15);
 drive.turn(right, imuTarget(90), 1, 70);
 
 drive.addErrorFunc(26, LAMBDA(drive.setMaxVelocity(70)));
 drive.move(forward, 37, 4, 40);

 drive.setPID(2);
 drive.setScheduleThreshold_a(NO_SCHEDULING);
 drive.setScheduleThreshold_l(NO_SCHEDULING);

 drive.turn(right, imuTarget(130), 1, 70);

 drive.move(backward, 22, 1, 100);

 stopIntake();

 drive.move(forward, 5, 1, 100);

 drive.turn(right, imuTarget(180), 1, 70);

 lift.move_voltage(12000);
 drive.addErrorFunc(2.5, LAMBDA(lift.move_voltage(0)));
 drive.move(backward, 5, 1, 100);

 startIntake();

 pros::delay(300);
 
 drive.addErrorFunc(8, LAMBDA(lift.move_voltage(-12000)));
 drive.addErrorFunc(2, LAMBDA(lift.move_voltage(0)));
 drive.move(forward, 12, 1, 100);
  
 drive.setPID(1);
 drive.setScheduledConstants(PIDConstants[4]);
 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.turn(left, imuTarget(26), 2, 70);

 drive.move(backward, 48, 2, 100);

 drive.setPID(2);
 drive.setScheduleThreshold_a(NO_SCHEDULING);
 drive.setScheduleThreshold_l(NO_SCHEDULING);
 drive.turn(left, imuTarget(310), 1, 70);
 
 drive.move(backward, 8, 1, 100);

 odomTask.remove();
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringSideRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.setScheduledConstants(PIDConstants[4]);

 drive.addErrorFunc(12, LAMBDA(drive.setMaxVelocity(25)));
 drive.move(backward, 36, 2, 100);

 mogoMechPisses.set_value(true);

 pros::delay(150); //let the clamp fully actuate

 conveyor.setIntake(400);
 startIntake();

 pros::delay(900); //give the pre load a moment 

                  /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,  130,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,   55,  0,  15,   0,  750,  0});
 drive.setScheduleThreshold_a(20);
 drive.setSlew(mogoProfile);

 drive.turn(right, imuTarget(110), 2, 70);
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 27, 2, 60);

 drive.turn(right, imuTarget(198), 2, 70);

 pros::delay(1000);

 drive.move(forward, 18, 1, 55);

 drive.move(backward, 30, 3, 100);

 drive.turn(left, imuTarget(68), 3, 70);

 drive.move(backward, 44, 3, 100);

 odomTask.remove();
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringSideBlue(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.setScheduledConstants(PIDConstants[4]);

 drive.addErrorFunc(12, LAMBDA(drive.setMaxVelocity(25)));
 drive.move(backward, 35, 2, 100);

 mogoMechPisses.set_value(true);

 pros::delay(150); //let the clamp fully actuate

 conveyor.setIntake(400);
 startIntake();

 pros::delay(900); //give the pre load a moment 

                  /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,  130,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,   55,  0,  15,   0,  750,  0});
 drive.setScheduleThreshold_a(20);
 drive.setSlew(mogoProfile);

 drive.turn(left, imuTarget(250), 2, 70);
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 27, 2, 60);

 drive.turn(left, imuTarget(162), 2, 70);

 pros::delay(1000);

 drive.move(forward, 20, 1, 55);

 drive.move(backward, 30, 3, 100);

 drive.turn(shortest, 301, 3, 70);

 drive.move(backward, 42, 3, 100);

 odomTask.remove();
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void goalSideRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.setScheduledConstants(PIDConstants[4]);

 drive.move(backward, 35, 2, 60);

 mogoMechPisses.set_value(true);

 pros::delay(150); //let the clamp fully actuate

 conveyor.setIntake(400);
 startIntake();

 pros::delay(1000); //give the pre load a moment 

                  /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,  130,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,   55,  0,  15,   0,  750,  0});
 drive.setScheduleThreshold_a(20);
 drive.setSlew(mogoProfile);

 drive.turn(left, imuTarget(252), 2, 70);
  
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 28, 2, 60);

 pros::delay(1500);

 drive.move(backward, 22, 2, 100);
 
 drive.turn(right, imuTarget(320), 2, 70);

 drive.move(backward, 25, 2, 100);

 stopIntake();

 odomTask.remove();
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void goalSideBlue(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);
 drive.setScheduledConstants(PIDConstants[4]);

 drive.move(backward, 34, 4, 65);

 mogoMechPisses.set_value(true);

 pros::delay(200); //let the clamp fully actuate

 conveyor.setIntake(350);
 startIntake();

 pros::delay(1000); //give the pre load a moment 

                  /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,  130,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,   55,  0,  15,   0,  750,  0});
 drive.setScheduleThreshold_a(20);
 drive.setSlew(mogoProfile);

 drive.turn(right, imuTarget(108), 2, 70);

 conveyor.setIntake(400);
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 25, 2, 60);

 pros::delay(1500);

 drive.move(backward, 22, 2, 100);
 
 drive.turn(left, imuTarget(40), 2, 70);

 drive.move(backward, 25, 3, 100);

 stopIntake();

 odomTask.remove();
 runOnError.remove();
 runIntakeControl.remove();
 drive.onErrorVector.clear();
}

void ringElimRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void ringElimBlue(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void goalElimRed(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}

void goalElimBlue(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}


void skills(){
 pros::Task odomTask(updateOdom_fn);
 pros::Task runOnError(onError_fn);
 pros::Task runIntakeControl(IntakeControlSystem_fn);

 conveyor.setIntake(400);
 drive.setPID(2);

 drive.move(backward, 3, 1, 100);

 mogoMechPisses.set_value(true);
 pros::delay(150);

 startIntake();

 drive.setSlew(mogoProfile);
 drive.setPID(4);
 drive.setScheduleThreshold_a(20);

 drive.move(backward, 10, 1, 100);

 pros::delay(800);

 //fuck rings 
                    /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,    100,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,   28,  0,  16,   0,  750,  0});
 drive.turn(shortest, 180, 1, 70);
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.setScheduleThreshold_l(10);
 drive.addErrorFunc(8, LAMBDA(drive.setMaxVelocity(80)));
 drive.move(forward, 30, 3, 100);

 pros::delay(800);
 
                  /*{kP,   kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,   250,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,  190,  0,  15,   0,  600,  0});

 drive.turn(left, imuTarget(90), 2, 70);
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 24, 2, 100);
 
 
                  /*{kP,   kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,   200,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,  120,  0,  15,   0,  300,  0});

 drive.turn(left, imuTarget(30), 1, 70);

 pros::delay(2000); //letrings get on just in case intake sell 
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.addErrorFunc(6, LAMBDA(drive.setMaxVelocity(80)));
 drive.move(forward, 30, 3, 100);

 drive.move(backward, 26, 3, 100);

                   /*{kP,   kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,    400,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,  200,  0,  15,   0,  300,  0});
 drive.turn(left, imuTarget(0), 2, 70);

 pros::delay(2000); //test
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.addErrorFunc(22, LAMBDA(drive.setMaxVelocity(80)));
 drive.move(forward, 44, 6, 100);
 
                   /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,   420,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}
 drive.setScheduledConstants({ 0,  190,  0,  15,   0,  700,  0});
 drive.turn(left, imuTarget(255), 2, 70);

 pros::delay(1500); //testing 

 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(backward, 20, 1, 100);

 mogoMechPisses.set_value(false);

 
 
 drive.setPID(1);
 drive.setScheduleThreshold_a(15);
 drive.setScheduledConstants(PIDConstants[4]);

 drive.move(forward, 32, 3, 100);
 
 drive.setPID(2);
 drive.setScheduleThreshold_a(NO_SCHEDULING);
 drive.turn(right, imuTarget(270), 1, 100);
 
 drive.setPID(1);
 drive.setScheduleThreshold_a(15);
 
 drive.move(forward, 48, 3, 100);


 stopIntake();
 */

 

 runOnError.remove();
 odomTask.remove();
 runIntakeControl.remove();
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
  //  drive.setPID(8);
  //  drive.setScheduleThresholds_s(0, 10);
  //  drive.setScheduledSwerveConstants(PIDConstants[6]);
  
  //  //170 degree angular movememt
  //  drive.swerve(forwardRight, 52, 40, 3, 60, 10);
  //  pros::delay(1000);

 drive.setPID(4);
 drive.setScheduleThreshold_a(20);
 drive.setScheduledConstants(PIDConstants[5]);

 imu.set_heading(180);

 drive.turn(left, imuTarget(26), 2, 70);


 /*
 drive.move(forward, 22, 1, 100);
 pros::delay(1000);
 drive.move(forward, 32, 2, 100);
 pros::delay(1000);
 drive.move(forward, 42, 2, 100);
 pros::delay(2000);
 */ 
 
 
 //  drive.turn(right, 50, 1, 70);
 //  pros::delay(1000);

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

 /*

 drive.turn(right, 195, 2, 70);
 pros::delay(1000);

 drive.turn(right, 205, 2, 70);
 pros::delay(1000);
    */

 

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}