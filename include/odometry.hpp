#include "main.h"
#include "pros/rotation.hpp"
#include "parametrics.hpp"

class Odometry{
 private:
 double wheelDiameter, verticalOffset, horizontalOffset;
 
 pros::Task *OdomTask = nullptr;
 float lastAngle = 0;

 float calcDeltaTheta(std::vector<std::shared_ptr<pros::IMU>> &imu, bool update = true);
 float calcDeltaTheta(pros::Rotation &tracker1, pros::Rotation &tracker2);

 public:
 Odometry(pros::Rotation &vertical, double verticalOffset, pros::Rotation &horizontal, double horizontalOffset,
            double wheelDiameter, pros::Imu &imu){
  this->vertical   = &vertical;
  this->horizontal = &vertical;
  this->imu        = &imu;
  this->verticalOffset = verticalOffset;
  this->horizontalOffset = horizontalOffset;
  this->wheelDiameter  = wheelDiameter;

 }
 
 pros::Rotation* vertical;
 pros::Rotation* horizontal;
 pros::Imu* imu;
 Pose pose = Pose(0, 0, 0);

 double degreeToInch(double deg);
 double inchToDegree(double inch);

 int getVertPos();
 int getHoriPos();

 void init();
 void calibrate(bool calibrateIMU = true);
 void update();
};