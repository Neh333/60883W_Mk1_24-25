#include "drive.hpp"
#include "parametrics.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include <memory>

double Odometry::degreeToInch(double deg) {
  return (deg * (360 / (wheelDiameter*M_PI)) );
}

double Odometry::inchToDegree(double inch) {
  return (inch / (360 / (wheelDiameter*M_PI)) );
}

/*rot getters*/
const int Odometry::getVertPos(){
  return (this->vertical->get_position()/100);
}

const int Odometry::getHoriPos(){
  return (this->horizontal->get_position()/100);
}

const int Odometry::getX(){
  return (this->pose.x);
}

const int Odometry::getY(){
  return (this->pose.y);
}

const Pose Odometry::getCurrentPose(){
  return (this->pose);
}

void Odometry::init(){
 if (OdomTask == nullptr) 
 {
    OdomTask = std::make_unique<pros::Task> ([this]() {
      while (true) 
      {
        this->update();
        pros::delay(20);
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

/*Prev vars*/
float prevVertical = 0;
float prevHorizontal = 0;
float prevImu = 0;

void Odometry::update(){

    /*get the current sensor values*/
    float verticalRaw = 0;
    float horizontalRaw = 0;
    float imuRaw = 0;

    verticalRaw = degreeToInch(getVertPos());
    horizontalRaw = degreeToInch(getHoriPos());
   
    degToRad(imu->get_rotation());

    // calculate the change in sensor values
    float deltaVertical = verticalRaw - prevVertical;
    float deltaHorizontal = horizontalRaw - prevHorizontal;
    float deltaImu = imuRaw - prevImu;

    // update the previous sensor values
    prevVertical = verticalRaw;
    prevHorizontal = horizontalRaw;
    prevImu = imuRaw;

    float heading = pose.theta;

    /* calculate the heading using the horizontal tracking wheels*/
    heading -=  deltaHorizontal / horizontalOffset;

    /*use the vertical tracking wheels*/
    heading -= deltaVertical / verticalOffset;

    /*use the IMU*/
    heading += deltaImu;

    float deltaHeading = heading - pose.theta;
    float avgHeading = pose.theta + deltaHeading / 2;

    // calculate change in x and y
    float deltaX = 0;
    float deltaY = 0;
    deltaY = verticalRaw - prevVertical;
    deltaX = horizontalRaw - prevHorizontal;

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { /* prevent divide by 0*/
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    // save previous pose
    Pose prevPose = pose;

    // calculate global x and y
    pose.x += localY * sin(avgHeading);
    pose.y += localY * cos(avgHeading);
    pose.x += localX * -cos(avgHeading);
    //pose += localX * sin(avgHeading); //look at what this does exactly and implementing
    pose.theta = heading;
}

double Drive::move_to(Direction dir, Coord targetPoint, double timeOut, double maxVelocity){
 /* Error values */
 double lastError;
 double errorDrift;
 double proportionDrift;
 const double initialHeading = imu->get_heading(); //inital theta
 const double initialVertical = odom->getVertPos();
 const double tickTarget = odom->inchToDegree((odom->getCurrentPose().distance(Pose(targetPoint))));
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
     error = tickTarget - fabs(odom->getVertPos() - initialVertical);
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
  double target = targetPoint.angle(odom->getCurrentPose());
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
 const double initalVertical = odom->getVertPos();
 const double tickTarget = odom->inchToDegree(odom->getCurrentPose().distance(Pose(targetPoint)));
 const double tickTargetCutOff = odom->inchToDegree(odom->getCurrentPose().distance(Pose(cutOffPoint)));
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
 while(tickTargetCutOff > fabs(odom->getVertPos() - initalVertical)){

      if(!scheduled && fabs(error) < scheduleThreshold_l){
      myKP = scheduledConstants.kP;
      myKI = scheduledConstants.kI;
      myKD = scheduledConstants.kD;
      scheduled = true;
     }

     error = tickTarget - fabs(odom->getVertPos() - initalVertical);
    
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
 double target_a = targetPose.angle(odom->getCurrentPose());
 
 const Pose initalPose = odom->getCurrentPose();

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
    const double d =  odom->getCurrentPose().distance(targetPose);

    /* calculate the carrot point 
    *
    *
    */
    Coord carrot(targetPose.x - d * cos(targetPose.theta) * dlead,
			           targetPose.y - d * sin(targetPose.theta) * dlead);

    error = odom->getCurrentPose().distance(carrot);


    
  } 
 /* Tell the onError task that the PID is over, then return the error at time of exit */
 moveDriveVoltage(0);
 isNewPID = false;
 return tickToInch(error);
}