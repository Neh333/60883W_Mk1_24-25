#include "drive.hpp"

/*CONSTRUCTORS*/
//Drive object constructor 
Drive::Drive(pros::MotorGroup &leftMotors, pros::MotorGroup &rightMotors, pros::Imu &imu){
 setPID(1);
 setScheduleThreshold_l(NO_SCHEDULING);
 setScheduleThreshold_a(NO_SCHEDULING);

 /* slew, slew_lower_thresh, slew_upper_thresh */
 setSlew({0, 30, 80});
 setSlew_a({0, 30, 80});

 this->leftMotors      = &leftMotors;
 this->rightMotors     = &rightMotors;
 this->imu             = &imu;
}

Drive::Drive(pros::MotorGroup &leftMotors, pros::MotorGroup &rightMotors, Odometry& odometry){
 setPID(1);
 setScheduleThreshold_l(NO_SCHEDULING);
 setScheduleThreshold_a(NO_SCHEDULING);

 /* slew, slew_lower_thresh, slew_upper_thresh */
 setSlew({0, 30, 80});
 setSlew_a({0, 30, 80});

 this->leftMotors      = &leftMotors;
 this->rightMotors     = &rightMotors;
 this->imu             = odometry.imu;
 this->odom = &odometry;
}

/*********************************************************************************************************/
/*DRIVE METHODS*/

double Drive::leftDriveAvgPos(){
  double value = 0;
  for (int i = 0; i<(leftMotors->size()); i++) {
    value += this->leftMotors->get_position(i);
  }
  return value/leftMotors->size();
}

double Drive::rightDriveAvgPos(){
   double value = 0;
   for (int i = 0; i<(rightMotors->size()); i++) {
    value += this->rightMotors->get_position(i);
  }
  return value/rightMotors->size();
}

double Drive::driveAvgPos(){
 return (leftDriveAvgPos()+rightDriveAvgPos())/2;
}

double Drive::actualVelocityLeft(){
 double value = 0;
 for (int i = 0; i<(leftMotors->size()-1); i++) {
    value += this->leftMotors->get_actual_velocity(i);
 }
 return value/leftMotors->size();
}

double Drive::actualVelocityRight(){
   double value = 0;
   for (int i = 0; i<rightMotors->size(); i++) {
    value += this->rightMotors->get_actual_velocity(i);
  }
  return value/rightMotors->size();
}

double Drive::actualVelocityAll(){
   return (actualVelocityLeft()+actualVelocityRight())/2;
}

void Drive::moveLeftDriveVoltage(int voltage) {
  leftMotors->move_voltage(voltage);
}

void Drive::moveRightDriveVoltage(int voltage) {
  rightMotors->move_voltage(voltage);
}

void Drive::moveDriveVoltage(int voltage) {
  moveLeftDriveVoltage(voltage);
  moveRightDriveVoltage(voltage);
}

void Drive::moveDriveTrain(int voltage, float time){
  moveDriveVoltage(voltage);
  pros::delay(time*1000);
  moveDriveVoltage(0);
}

void Drive::setBrakeMode(pros::motor_brake_mode_e brakeMode) {
  leftMotors->set_brake_mode_all(brakeMode);
  rightMotors->set_brake_mode_all(brakeMode);
}