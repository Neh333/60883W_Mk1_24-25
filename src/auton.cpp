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
  {tune}
};

//weird profiles for reuse 
PIDprofile deg90RingConsts{ 0,   250,  0,   0,   0,  200,    0};
PIDprofile deg90RingScheduledConsts{ 0,  190,  0,  15,   0,  600,  0};
PIDprofile deg60RingConsts{ 0,   200,  0,   0,   0,  200,    0};
PIDprofile deg60RingSchedulesConsts{0,  125,  0,  15,   0,  320,  0};
PIDprofile deg110RingConsts{ 0,   250,  0,   0,   0,  200,    0};
PIDprofile deg110RingScheduledConsts{ 0,  190,  0,  15,   0,  600,  0};
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
 pros::delay(400); // give rimg some time

 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]); 

 drive.addErrorFunc(22, LAMBDA(mogoMechPisses.set_value(false)));
 drive.addErrorFunc(20, LAMBDA(drive.setPID(1)));
 drive.addErrorFunc(20, LAMBDA(drive.setScheduledConstants(PIDConstants[4])));
 drive.addErrorFunc(20, LAMBDA( drive.setSlew({0,0,0})));
 drive.move(forward, 44, 2, 100);
 
 drive.setScheduleThreshold_a(15);
 drive.turn(right, imuTarget(90), 1, 70);
 
 drive.setPID(2);
 drive.setScheduleThreshold_l(NO_SCHEDULING);
 drive.move(forward, 14, 1, 100); 

 drive.setPID(1);
 drive.setScheduledConstants(PIDConstants[4]);
 drive.setScheduleThreshold_l(10);
 drive.move(forward, 26, 3, 70); 

 pros::delay(1100); //outake red

 stopIntake();

 //turn to secong mogo
 drive.turn(left, imuTarget(338), 1, 70);

 drive.addErrorFunc(8, LAMBDA(startIntake()));
 drive.move(backward, 28, 3, 100);

 mogoMechPisses.set_value(true);
 pros::delay(150);
 
 drive.setSlew(mogoProfile);
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.setScheduleThreshold_a(20);
 drive.move(backward, 20, 2, 100);

 mogoMechPisses.set_value(false);
 pros::delay(100);
  
 drive.setSlew({0,0,0});
 drive.setPID(2);
 drive.setScheduleThreshold_l(NO_SCHEDULING);
 drive.move(forward, 8, 1, 100);

 drive.setScheduleThreshold_l(10);
 drive.setScheduleThreshold_a(15);
 drive.setScheduledConstants(PIDConstants[4]);
 drive.turn(right, imuTarget(47), 1, 70); 

 drive.setSlew({0,0,0});
 drive.setPID(2);
 drive.setScheduleThreshold_l(NO_SCHEDULING);
 drive.move(backward, 13, 1, 100);

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

 drive.move(backward, 52, 10, 100);

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

 drive.move(backward, 34, 2, 60);

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

 drive.move(backward, 26, 2, 100);

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
 drive.move(forward, 28, 2, 60);

 pros::delay(1500);

 drive.move(backward, 22, 2, 100);
 
 drive.turn(left, imuTarget(40), 2, 70);

 drive.move(backward, 26, 3, 100);

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

 pros::delay(1000); // intake copuim

 stopIntake(); //so it won't roll ring off 

 drive.setSlew(mogoProfile);
 drive.setPID(4);
 drive.setScheduleThreshold_a(20);

 drive.move(backward, 10, 1, 100);

 //fuck rings 
 drive.setCustomPID({ 0,    100,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,   28,  0,  16,   0,  750,  0});
 drive.turn(shortest, 180, 1, 70);

 startIntake(); //pray 
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.setScheduleThreshold_l(10);
 drive.addErrorFunc(8, LAMBDA(drive.setMaxVelocity(80)));
 drive.move(forward, 30, 3, 100);

 pros::delay(1000); //rings are pain 
 
 drive.setCustomPID(deg90RingConsts);
 drive.setScheduledConstants(deg90RingScheduledConsts);
 drive.turn(left, imuTarget(90), 2, 70);
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 24, 2, 100);
 
 
 drive.setCustomPID(deg60RingConsts);
 drive.setScheduledConstants(deg60RingSchedulesConsts);
 //58 deg turn 
 drive.turn(left, imuTarget(32), 1, 70);

 pros::delay(1500); //let rings get on just in case intake sell 
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.addErrorFunc(6, LAMBDA(drive.setMaxVelocity(70)));
 drive.move(forward, 30, 3, 100);

 drive.move(backward, 26, 3, 100);

                   /*{kP,   kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({ 0,    450,  0,   0,   0,  200,    0});
                           /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setScheduledConstants({ 0,  220,  0,  0,   0,  300,  0});
 drive.turn(left, imuTarget(0), 2, 70);

 pros::delay(1000); //let rings go on 
 
 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 24, 3, 50);

 pros::delay(1000); //get first ring 

 drive.move(forward, 12, 2, 50); //get 2nd ring 

 pros::delay(1000); //let last ring get on conveyer

                   
 drive.setCustomPID(deg110RingConsts);
 drive.setScheduledConstants(deg110RingScheduledConsts);
 drive.turn(left, imuTarget(248), 2, 100);

 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(backward, 20, 2, 100);

 pros::delay(1300); //let everything score
 
 //drop off 1st mogo 
 mogoMechPisses.set_value(false);
 pros::delay(200);

 //end of 1st part
 


 //go for 2nd mogo 
 drive.setPID(2);
 drive.setScheduleThreshold_l(NO_SCHEDULING);
 drive.setScheduleThreshold_a(NO_SCHEDULING);
 drive.move(forward, 6, 1, 100);
 
 //20 deg turn go brrrrr
                  /*{kP,  kPa, kI, kIa,  kD,  kDa,  kPd}*/
 drive.setCustomPID({0,   212,  0,   0,   0,  390,   0});
 drive.turn(right, imuTarget(272), 2, 100);
 

 //kpd prob being weird 
 drive.setCustomPID({25,  340,  0,  0, 100,  40,    0});
 drive.setScheduledConstants(PIDConstants[4]);
 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);

 drive.move(forward, 69, 6, 100);
 
 //hates me 
 drive.turn(right, imuTarget(58), 2, 70);

 drive.setPID(2);
 drive.setScheduleThreshold_l(NO_SCHEDULING);
 drive.setScheduleThreshold_a(NO_SCHEDULING);
 drive.move(backward, 18, 1, 100);

 mogoMechPisses.set_value(true);
 pros::delay(150);

 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.setScheduleThreshold_l(10);
 drive.setScheduleThreshold_a(20);
 drive.setSlew(mogoProfile);

 drive.move(backward, 10, 1, 100);
 
 drive.setCustomPID(deg110RingConsts);
 drive.setScheduledConstants(deg110RingScheduledConsts);
 drive.turn(left, imuTarget(303), 2, 70);

 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 24, 2, 70);

 pros::delay(800); //let ring on conveyer 

 drive.turn(left, imuTarget(240), 2, 70); 

 drive.move(forward, 18, 2, 70);

 pros::delay(800); //get ring on conveyer 

 drive.move(backward, 16, 2, 100);
 
 //get two in line 
 //60 deg turn 
 drive.setCustomPID(deg60RingConsts);
 drive.setScheduledConstants(deg60RingSchedulesConsts);
 drive.turn(left, imuTarget(180), 2, 70); 

 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);
 drive.move(forward, 20, 3, 50);

 pros::delay(1000); //get first ring 

 drive.move(forward, 16, 2, 70); //get 2nd ring 
 
 //90 deg turn 
 drive.setCustomPID(deg90RingConsts);
 drive.setScheduledConstants(deg90RingScheduledConsts);
 drive.turn(left, imuTarget(270), 2, 70);

 drive.setPID(4);
 drive.setScheduledConstants(PIDConstants[5]);

 drive.move(forward, 26, 3, 60);

 pros::delay(500);
 
 //40 deg turn 
 drive.turn(right, imuTarget(230), 2, 70);
 pros::delay(1000);
 
 drive.addErrorFunc(8, LAMBDA(drive.setMaxVelocity(60)));
 drive.move(forward, 48, 4, 100);

 drive.turn(left, imuTarget(175), 2, 70);

 drive.move(backward, 72, 7, 100);

 mogoMechPisses.set_value(false);
 pros::delay(200);

 drive.setPID(1);
 drive.setScheduledConstants(PIDConstants[4]);
 drive.move(forward, 24, 1, 100);


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
 drive.setPID(1);
 drive.setScheduledConstants(PIDConstants[4]);
 drive.setScheduleThreshold_a(15);
 drive.setScheduleThreshold_l(10);

 drive.turn(right, 158, 2, 70);
 pros::delay(2000);
 /*
 drive.move(forward, 22, 1, 100);
 pros::delay(1000);
 drive.move(forward, 32, 2, 100);
 pros::delay(1000);
 drive.move(forward, 42, 2, 100);
 pros::delay(2000);
 
 
 
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

 drive.turn(right, 195, 2, 70);
 pros::delay(1000);

 drive.turn(right, 205, 2, 70);
 pros::delay(1000);
    */

 

 odomTask.remove();
 runOnError.remove();
 drive.onErrorVector.clear();
}