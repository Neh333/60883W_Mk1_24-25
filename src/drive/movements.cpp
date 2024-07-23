#include "drive.hpp"
#include "util.hpp"
#include <cmath>
#include <math.h>
#include "include.hpp"
#include <algorithm>
 
/* Basic linear PID movement function */
double Drive::move(Direction dir, double target, double timeOut, double maxVelocity){
  /* Error values */
  double lastError;
  double errorDrift;
  double proportionDrift;
  const double initialHeading = imu->get_heading();
  const double initialMotorAvg = driveAvgPos();
  const double tickTarget = inchToTick(target);
  /* Scheduling variables */
  bool scheduled = (scheduleThreshold_l == NO_SCHEDULING);
  double myKP = this->kP, myKI = this->kI, myKD = this->kD;
  /* Integral declaration */
  double integral = 0;
  /* Motor output variable declarations */
  maxVolt = percentToVoltage(maxVelocity);
  double finalVolt;
  /* Drive output multiplier */
  const int8_t reverseVal = (dir == backward)?(-1):(1);
  /* Standstill variable declarations */
  uint8_t standStillCount = 0;
  bool standStill = false;
  /* Tell the onError task that a new PID has begun, and set endTime */
  isNewPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while(pros::millis() < endTime && !standStill)
  {
        /* Maybe schedule constants */
    if(!scheduled && fabs(error) < scheduleThreshold_l)
    {
      myKP = scheduledConstants.kP;
      myKI = scheduledConstants.kI;
      myKD = scheduledConstants.kD;
      scheduled = true;
    }

    /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
    error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
    finalVolt = updatePID(myKP, myKI, myKD, error, lastError, integral, integralActive);
    calculateSlew(&finalVolt, actualVelocityAll(), &slewProf);
    finalVolt = std::clamp(finalVolt, -maxVolt, maxVolt);

    // Print statement used for testing
    controller.print(2, 0, "Error: %.4f", tickToInch(error));

    // Check if robot is in standstill and updates the standstill flag accordingly
    updateStandstill(lateral, standStill, error, lastError, standStillCount);

    /* Update lastError */
    lastError = error;
  
    /* Calculate the product of heading drift and kP_d */
    errorDrift = fmod((initialHeading-(imu->get_heading())+540),360) - 180;
    proportionDrift = errorDrift * kP_d;

    /* Move Drivetrain */
    moveRightDriveVoltage((reverseVal*finalVolt)+proportionDrift);
    moveLeftDriveVoltage((reverseVal*finalVolt)-proportionDrift);
    
    /* Give PROS time to keep itself in order */
    pros::delay(20);
  }

  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  isNewPID = false;
  return tickToInch(error);
}


/* Basic angular PID movement function */
double Drive::turn(Direction dir, double target, double timeOut, double maxVelocity)
{ 
  double lastError;
  const double initialAngle = imu->get_rotation() + 360;
  /* Scheduling variables */
  bool scheduled = (scheduleThreshold_a == NO_SCHEDULING);
  double myKP = this->kP_a, myKI = this->kI_a, myKD = this->kD_a;
  /* Integral definition */
  double integral = 0;
  /* Motor output variable declarations */
  maxVolt_a = percentToVoltage(maxVelocity);
  double finalVolt;
  /* Drive output multiplier */
  int8_t reverseVal = (dir == right)?(1):(-1);
  /* Standstill variable declarations */
  uint8_t standStillCount = 0;
  bool standStill = false;

  /* Change the reverseVal and target if the direction input is shortest */
  if(dir == shortest)
  {
    target = fabs(fmod((target-imu->get_heading()+540),360) - 180);
    reverseVal = sgn(target);
    target = fabs(target);
  }
  
  /* Tell the onError task that a new PID has begun, and set endTime */
  isNewPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while((pros::millis() < endTime && !standStill)){
    if(!scheduled && fabs(error) < scheduleThreshold_a)
    {
      myKP = scheduledConstants.kP_a;
      myKI = scheduledConstants.kI_a;
      myKD = scheduledConstants.kD_a;
      scheduled = true;
    }

    /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
    error = target - fabs(imu->get_rotation() + 360 - initialAngle);
    finalVolt = updatePID(myKP, myKI, myKD, error, lastError, integral, integralActive_a);
    calculateSlew(&finalVolt, actualVelocityLeft() - actualVelocityRight(), &slewProf_a);
    finalVolt = std::clamp(finalVolt, -maxVolt_a, maxVolt_a);

    // Print statement used for testing
    controller.print(2, 0, "Error: %.4f", error);

    /* Calculate standstill */
    updateStandstill(lateral, standStill, error, lastError, standStillCount);
    
    /* Update lastError */
    lastError = error;

    /* Move Drivetrain */
    moveRightDriveVoltage(reverseVal*-finalVolt);
    moveLeftDriveVoltage(reverseVal*finalVolt);
    
    /* Give PROS time to keep itself in order */
    pros::delay(20);
  }

  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  isNewPID = false;
  return error;
}

double Drive::hardStop(Direction dir, double targetCutOff, double target, double maxVelocity){
  double errorDrift;
  double proportionDrift;
  double lastError;
  const double initialHeading = imu->get_heading();
  const double initialMotorAvg = driveAvgPos();
  const double tickTarget = inchToTick(target);
  const double tickTargetCutOff = inchToTick(targetCutOff);
  /* Scheduling variables */
  bool scheduled = (scheduleThreshold_l == NO_SCHEDULING);
  double myKP = this->kP, myKI = this->kI, myKD = this->kD;
  //Motor output var declarations//
  maxVolt = percentToVoltage(maxVelocity);
  double finalVolt;
 
  /* Drive output multiplier */
  const int8_t reverseVal = (dir == backward)?(-1):(1);
   
  /*Tell the onError task that a new PID has begun*/
  isNewPID = true;

  //Begin PID
  while(tickTargetCutOff > fabs(driveAvgPos()-initialMotorAvg)){
    /* Maybe schedule constants */
    if(!scheduled && fabs(error) < scheduleThreshold_l){
      myKP = scheduledConstants.kP;
      myKI = scheduledConstants.kI;
      myKD = scheduledConstants.kD;
      scheduled = true;
     }
     error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
    
     //update the PD values
     double zero = 0;
     finalVolt = updatePID(myKP, 0, myKD, error, lastError, zero, 0);
     
     //print error value for tuning
     controller.print(2,0, "Error: %.4f", tickToInch(error));
     
     /* Calculate the product of heading drift and kP_d */
     errorDrift = fmod((initialHeading-imu->get_heading()+540),360) - 180;
     proportionDrift = errorDrift * kP_d;

     //Set Drivetrain
     moveRightDriveVoltage((reverseVal*finalVolt)+proportionDrift);
     moveLeftDriveVoltage((reverseVal*finalVolt)-proportionDrift);

     //Give PROS time to keep itself in order
     pros::delay(20);
  }
  //Set voltage to 0 in case this is the last function called in an autonomous
  moveDriveVoltage(0);
  //Tell the onError task that the PID is over
  isNewPID = false;
  //Exit the function
  return tickToInch(error);
}

//TO DO: potentailly add scheduling 

/* Swerve movement */
double Drive::swerve(Direction dir, double target, double target_a, double timeOut, double maxVel, double maxVel_a){
  /* Error values */
  double error_a;
  double lastError;
  double lastError_a;
  const double initialMotorAvg = driveAvgPos();
  const double tickTarget = inchToTick(target);
  const double initialAngle = imu->get_rotation() + 360;
  /* Integral declarations */
  double integral = 0;
  double integral_a = 0;
  /* Motor output variable declarations */
  maxVolt = percentToVoltage(maxVel);
  maxVolt_a = percentToVoltage(maxVel_a);;
  double workingVolt;
  double finalVoltLeft;
  double finalVoltRight;
  /* Drive output multipliers */
  const int8_t reverseVal = (dir == backwardLeft || dir == backwardRight || dir == backwardShortest)?(-1):(1);
  int8_t reverseVal_a = (dir == backwardRight || dir == forwardRight)?(1):(-1);

  /* Change the reverseVal and target if the direction input is shortest */
  if(dir == forwardShortest || dir == backwardShortest)
  {
    target_a = fabs(fmod((target-imu->get_heading()+540),360) - 180);
    reverseVal_a = sgn(target_a);
    target_a = fabs(target_a);
  }

  /* Standstill variable declarations */
  uint8_t standStillCount = 0;
  uint8_t standStillCount_a = 0;
  bool standStill = false;
  bool standStill_a = false;

  /* Tell the onError task that a new PID has begun, and set endTime */
  isNewPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while(pros::millis() < endTime && !(standStill && standStill_a))
  {
    /********************************************DRIVE***************************************************/
    /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
    error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
    workingVolt = updatePID(kP, kI, kD, error, lastError, integral, integralActive);
    calculateSlew(&workingVolt, actualVelocityAll(), &slewProf);
    workingVolt = std::clamp(workingVolt, -maxVolt, maxVolt);

    /* Calculate standstill */
    updateStandstill(lateral, standStill, error, lastError, standStillCount);
    
    /* Update lastError */
    lastError = error;
  
    finalVoltRight = reverseVal*workingVolt;
    finalVoltLeft = reverseVal*workingVolt;
    /********************************************TURN****************************************************/
    /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
    error_a = target_a - fabs(imu->get_rotation() + 360 - initialAngle);
    workingVolt = updatePID(kP_a, kI_a, kD_a, error_a, lastError_a, integral_a, integralActive_a);
    calculateSlew(&workingVolt, actualVelocityLeft() - actualVelocityRight(), &slewProf_a);
    workingVolt = std::clamp(workingVolt, -maxVolt_a, maxVolt_a);

    

    /* Calculate standstill */
    updateStandstill(lateral, standStill_a, error_a, lastError_a, standStillCount_a);
    
    /* Update lastError */
    lastError_a = error_a;
    
    finalVoltRight += reverseVal_a*(-workingVolt);
    finalVoltLeft += reverseVal_a*(workingVolt);
    /********************************************MOVE****************************************************/
    //print error value for tuning
    //controller.print(2,0,"E: %.4f, EA: %.4f", tickToInch(error), error_a);
    //controller.print(2,0, "EA: %.4f", error_a);
    controller.print(2,0, "Error: %.4f", tickToInch(error));

    /* Move Drivetrain */
    moveRightDriveVoltage(finalVoltRight);
    moveLeftDriveVoltage(finalVoltLeft);

    /* Give PROS time to keep itself in order */
    pros::delay(20);
  }
  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  isNewPID = false;
  return tickToInch(error);
}
  
/* Actively halt robot movement for timeOut seconds */
double Drive::brake(double timeOut)
{
  const double target = driveAvgPos();
  const double kP_brake = 30;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  while(pros::millis() < endTime)
  {
      error = target - driveAvgPos();
      moveDriveVoltage(error*kP_brake);
      pros::delay(20);
  }

  return tickToInch(error);
}