#include "drive.hpp"
#include "parametrics.hpp"
#include "include.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "util.hpp"
#include <cmath>

Pose odomPose = Pose(0,0,0);

double degreeToInch(double deg) {
  return (deg * ((2*M_PI)/360) );
}

double inchToDegree(double inch) {
  return (inch / ((2*M_PI)/360) );
}

void updateOdom_fn(void *param){

 horizontalTracker.reset_position();
 verticalTracker.reset_position();

 double prevVertical = 0;
 double prevHorizontal = 0;
 double prevImu = 0;
 double prevTheta = 0;

 while (true) {
 /* Get the current sensor values */
    float verticalRaw = degreeToInch(verticalTracker.get_position()/100);
    float horizontalRaw = degreeToInch(horizontalTracker.get_position()/100);
    float imuRaw = degToRad(imu.get_rotation());

    // Calculate the change in sensor values
    float deltaVertical = verticalRaw - prevVertical;
    float deltaHorizontal = horizontalRaw - prevHorizontal;
    float deltaImu = imuRaw - prevImu;

    // Update the previous sensor values
    prevVertical = verticalRaw;
    prevHorizontal = horizontalRaw;
    prevImu = imuRaw;

    float heading = odomPose.theta;

    /* Calculate the heading using the horizontal tracking wheels */
    //heading -= deltaHorizontal / horizontalOffset;

    /* Use the vertical tracking wheels */
    //heading -= deltaVertical / verticalOffset;

    /* Use the IMU */
    //heading += deltaImu;

    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    // Calculate change in x and y
    float deltaX = deltaHorizontal;
    float deltaY = deltaVertical;

    // Calculate local x and y
    float localX, localY;

    if (fabs(deltaHeading) < 0) { // Prevent division by zero
        localX = deltaX;
        localY = deltaY;
    } 
    
    else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading /*+ horizontalOffset*/);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading /*+ verticalOffset*/);
    }

    // Calculate global x and y
    odomPose.x += localX * cos(avgHeading) - localY * sin(avgHeading);
    odomPose.y += localX * sin(avgHeading) + localY * cos(avgHeading);

    odomPose.theta = heading;

    pros::lcd::print(0, "Vert Val: %.3f", verticalTracker.get_position()/100);
    pros::lcd::print(2, "Hori Val: %.3f", horizontalTracker.get_position()/100);
    pros::lcd::print(4, "X Val: %.3f", odomPose.x);
    pros::lcd::print(6, "Y Val: %.3f", odomPose.y);

    pros::delay(20);
 }
}
 

double Drive::move_to(Direction dir, Coord targetPoint, double timeOut, double maxVelocity){
 /* Error values */
 double lastError;
 double errorDrift;
 double proportionDrift;
 const double initialHeading = imu->get_heading(); //inital theta
 const double initialVertical = verticalTracker.get_position();
 const double tickTarget = inchToDegree((odomPose.distance(Pose(targetPoint))));
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
 while(pros::millis() < endTime && !standStill){
     /* Maybe schedule constants */
     if(!scheduled && fabs(error) < scheduleThreshold_l)
     {
      myKP = scheduledConstants.kP;
      myKI = scheduledConstants.kI;
      myKD = scheduledConstants.kD;
      scheduled = true;
     }

     /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
     error = tickTarget - fabs(verticalTracker.get_position() - initialVertical);
     finalVolt = updatePID(myKP, myKI, myKD, error, lastError, integral, integralActive);
     calculateSlew(&finalVolt, actualVelocityAll(), &slewProf);
     finalVolt = std::clamp(finalVolt, -maxVolt, maxVolt);

     /* Print statement used for testing */
     controller.print(2, 0, "Error: %.2f", tickToInch(error));

     /* Check if robot is in standstill and updates the standstill flag accordingly */
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

double Drive::turn_to(Direction dir, Coord targetPoint, double timeOut, double maxVelocity){
  double lastError;
  double target = targetPoint.angle(odomPose);
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
    controller.print(2, 0, "Error:     %.2f", error);

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

double Drive::hardStop_at(Direction dir, Coord cutOffPoint, Coord targetPoint, double maxVelocity){
 double errorDrift;
 double proportionDrift;
 double lastError;
 const double initialHeading = imu->get_heading();
 const double initalVertical = verticalTracker.get_position();
 const double tickTarget = inchToDegree(odomPose.distance(Pose(targetPoint)));
 const double tickTargetCutOff = inchToDegree(odomPose.distance(Pose(cutOffPoint)));
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
 while(tickTargetCutOff > fabs(verticalTracker.get_position()) - initalVertical){

      if(!scheduled && fabs(error) < scheduleThreshold_l){
      myKP = scheduledConstants.kP;
      myKI = scheduledConstants.kI;
      myKD = scheduledConstants.kD;
      scheduled = true;
     }

     error = tickTarget - fabs(verticalTracker.get_position() - initalVertical);
    
     //update the PD values
     double zero = 0;
     finalVolt = updatePID(myKP, zero, myKD, error, lastError, zero, zero);
     
     //print error value for tuning
     controller.print(2,0, "Error: %.2f", tickToInch(error));
     
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


//TO DO: FINISH                 
double Drive::swerve_To(Direction dir, Pose targetPose, double timeOut, double maxVel, double maxVel_a, double dlead){
 /* Error values */
 double error_a;
 double lastError;
 double lastError_a;
 double target_a = targetPose.angle(odomPose);
 
 const Pose initalPose = odomPose;

 /* Scheduling variables */
 bool scheduled = swerveThresholds.first == NO_SCHEDULING;
 bool scheduled_a = swerveThresholds.second == NO_SCHEDULING;
 double myKP = this->kP, myKI = this->kI, myKD = this->kD;
 double myKP_a = this->kP_a, myKI_a = this->kI_a, myKD_a = this->kD_a;

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
 if(dir == forwardShortest || dir == backwardShortest){
   target_a = fabs(fmod((target_a-imu->get_heading()+540),360) - 180);
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
 while(pros::millis() < endTime && !(standStill && standStill_a)) {

   if(!scheduled && fabs(error) < swerveThresholds.first){
     myKP = scheduledSwerveConstants.kP;
     myKI = scheduledSwerveConstants.kI;
     myKD = scheduledSwerveConstants.kD;
     scheduled = true;
    }

    if(!scheduled && fabs(error_a) < swerveThresholds.second){
      myKP_a = scheduledSwerveConstants.kP_a;
      myKI_a = scheduledSwerveConstants.kI_a;
      myKD_a = scheduledSwerveConstants.kD_a;
      scheduled_a = true;
    }
    
    /*check to make sure if d needs to use init or current and adjust acordingly*/
    const double d =  odomPose.distance(targetPose);

    /* calculate the carrot point 
    *
    *
    */
    Coord carrot(targetPose.x - d * cos(targetPose.theta) * dlead,
			           targetPose.y - d * sin(targetPose.theta) * dlead);

    error = odomPose.distance(carrot);


    
  } 
 /* Tell the onError task that the PID is over, then return the error at time of exit */
 moveDriveVoltage(0);
 isNewPID = false;
 return tickToInch(error);
}