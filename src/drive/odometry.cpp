#include "drive.hpp"
#include "parametrics.hpp"
#include "include.hpp"


void Odometry::init(){
 if (OdomTask == nullptr) {
     OdomTask = new pros::Task([this]() {
     while (true) {
      update();
      pros::delay(10);
     }
     });
    }
}

void Odometry::calibrate(bool calibrateIMU){
 if (!calibrateIMU) {
   vertical->reset();
   horizontal->reset();
 }
 else {
  imu->reset();
  vertical->reset();
  horizontal->reset();
 }
}

void Odometry::update(){
    double lastX, lastY;
    double x, y;
    double theta;

    this->pose = Pose(x,y,theta);
}

double Drive::move_to(Direction dir, Coord targetPoint, double timeOut, double maxVelocity){
 /* Error values */
 double lastError;
 double errorDrift;
 double proportionDrift;
 const double initialHeading = imu->get_heading();
 const double initialMotorAvg = driveAvgPos();
 const double tickTarget = inchToTick(odom.pose.distance(Pose(targetPoint)));
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
     error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
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
  double target = targetPoint.angle(odom.pose);
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
 const double initialMotorAvg = driveAvgPos();
 const double tickTarget = inchToTick(odom.pose.distance(Pose(targetPoint)));
 const double tickTargetCutOff = inchToTick(odom.pose.distance(Pose(cutOffPoint)));
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
                 
double Drive::swerve_To(Direction dir, Pose targetPose, double timeOut, double maxVel, double maxVel_a){

}