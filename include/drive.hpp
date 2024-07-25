  #pragma once
#include "main.h"
#include "parametrics.hpp"
#include "odometry.hpp"
#include "util.hpp"
#include <utility>

//better than a macro soo
const double INTEGRAL_MAX = 800.0;
const double NO_SCHEDULING = -1.f;

enum Direction{
  forward,
  backward,
  left,
  right,
  shortest,
  forwardRight,
  forwardLeft,
  forwardShortest,
  backwardRight,
  backwardLeft,
  backwardShortest
};

// Flags used for standstill calculations
enum movement_Type {
  lateral,
  turn_type
};


struct errorFuncTuple
{
  std::function<void()> func;
  double onError;
  bool called;

  errorFuncTuple(std::function<void()> func, double onError, bool called) : 
   func(func), onError(onError), called(called){}
};

void onError_fn(void* param);

struct PIDprofile
{
  double kP;
  double kP_a;
  double kI;
  double kI_a;
  double kD;
  double kD_a;
  double kP_d;
};

struct slewProfile
{
  double slew;
  double slew_lower_thresh;
  double slew_upper_thresh;
};

// Initialize PID Values
const PIDprofile PIDConstants[9] = {
/*{kP, kPa, kI, kIa, kD,  kDa,  kPd}*/
  {25,  340,  0,  0, 100,  40,    40},/*init prfoile of 60+ degree turns / lateral 22+*/

  {20,  152,  0,  13, 100, 382,   40},/*40+ degree turns / 2+ inch lateral (no scheduling)*/ 

  {25,    0,  0,   0,  70,  30,   40}, /* 10+ degree turns, 40+ inch normal weight (no scheduling)*/ 

  {20,  400,  0,   0, 120,   100, 20}, /*mogo turning 60+ degree turns, mogo lateral 22+*/ 
 
  /***********Scheduled**************/
   
  {20,  150, 0,  0, 150, 430,      0},/*scheduled for profile 1, starting at 15 deg and 10 in of error*/

  {30,  140, 0, 18, 100, 547,     20},/*scheduled for profile 4, starting at 20 deg and 10 in of error*/

  {10,   90,  0, 0,   0,  400,     0},/*scheduled for profile 8, at 15 deg & 10 in of erorr*/
 
  /***********SWERVES**************/

  {21,  200,  0, 0,  50,  130,     0},/*AWP swerve*/
};

class Drive{
 private:

 double kP, kP_a, kI, kI_a, kD, kD_a, kP_d;

 PIDprofile scheduledConstants, scheduledSwerveConstants;
 
 double scheduleThreshold_l, scheduleThreshold_a; 

 std::pair<double, double> swerveThresholds; //{L, A}
  
 double error;
  
 const double integralActive = inchToTick(3);
 const double integralActive_a = 15;

 double maxVolt;
 double maxVolt_a;
  
 /*Standstill variable declerations*/
 double maxStepDistance = 2;
 double maxStepTurn     = 0.2;

 double SSMaxCount   = 7;
 double SSMaxCount_t = 7;

 bool SSActive   = true;
 bool SSActive_t = true;
  
 /*on error flag*/
 bool isNewPID = false;
  
 /*PID updater methods*/ 
 double updatePID(double KP, double KI, double KD, double error, double lastError, double &integral, 
                 double integralActive);

 void updateIntegral(double error, double lastError, double activeDistance, double& integral);

 void updateStandstill(movement_Type type, bool& standStill, double error, double lastError,
                         uint8_t& standStillCount);

 void calculateSlew(double *voltage, double actualVelocity, slewProfile *profile);

 /*"Virtual" Drivetrain methods*/
 double leftDriveAvgPos(); 
 double rightDriveAvgPos();
 double driveAvgPos();

 /*returns a float between 0 and 100, representing how much energy is being used compared to actual movement*/
 double driveEfficiencyAll();

 struct slewProfile slewProf;
 struct slewProfile slewProf_a;

 public:
 /*Drive object constructor*/ 
 Drive(pros::MotorGroup &leftMotors, pros::MotorGroup &rightMotors, pros::Imu &imu);

 /*"Virtual" Drivetrain attributes and methods*/ 
 pros::MotorGroup *rightMotors;
 pros::MotorGroup *leftMotors;
 pros::Imu        *imu;

 void setBrakeMode(pros::motor_brake_mode_e brakeMode);;

 void moveLeftDriveVoltage(int voltage);
 void moveRightDriveVoltage(int voltage);
 void moveDriveVoltage(int voltage); 

 void moveDriveTrain(int voltage, float time);

 /*setters*/
 void setMaxVelocity(float velocity);
 void setMaxTurnVelocity(float velocity);
 void setCustomPID(PIDprofile profile);

 void setScheduledConstants(PIDprofile constants);
 void setScheduledSwerveConstants(PIDprofile constants);

 void setScheduleThreshold_l(double error);
 void setScheduleThreshold_a(double error);
 void setScheduleThresholds_s(double error, double error_a);

 void setSlew(slewProfile profile);
 void setSlew_a(slewProfile profile);
 void setStandStill(movement_Type type, uint8_t maxCycles, float largestStep);
 void setPID(uint8_t n);
  
 /*Getters*/
 const double getError();
 const bool getPIDStatus();

 /*returns a float between -200 and 200*/
 double actualVelocityLeft();
 double actualVelocityRight();
 double actualVelocityAll();

 /* The onError vector */
 std::vector<errorFuncTuple> onErrorVector;
 void addErrorFunc(double onError, void input());

 /* Movement Functions, Return error after movement is finished */
 double move(Direction dir, double target, double timeOut, double maxVelocity);
 double turn(Direction dir, double target, double timeOut, double maxVelocity);

 /*Hardstop fn for PID motion chaining*/ 
 double hardStop(Direction dir, double targetCutOff, double target, double maxVelocity);
  
 /*Swerve Movemnet Function*/                 
 double swerve(Direction dir, double target, double target_a, double timeOut, double maxVel, 
                 double maxVel_a);
 
 /*P Based brake loop*/
 double brake(double timeOut);
 
 /********************************************ODOMETRY MOVRMENTS*************************************************************/

 /* Movement Functions, Return error after movement is finished */
 double move_to(Direction dir, Coord targetPoint, double timeOut, double maxVelocity);
 double turn_to(Direction dir, Coord targetPoint, double timeOut, double maxVelocity);

 /*Hardstop fn for PID motion chaining*/ 
 double hardStop_at(Direction dir, Coord cutOffPoint, Coord targetPoint, double maxVelocity);
  
 /*Swerve Movemnet Function*/                 
 double swerve_To(Direction dir, Pose targetPose, double timeOut, double maxVel, double maxVel_a, double dlead);

};

/*Drive object instance declartion*/
extern Drive drive;

