#include "main.h"
#include "pros/rotation.hpp"
#include "parametrics.hpp"
class Odometry{
 private:
 pros::Rotation* vertical;
 pros::Rotation* horizontal;
 double wheelSize;
 
 pros::Task *OdomTask = nullptr;
 float lastAngle = 0;

 float calcDeltaTheta(std::vector<std::shared_ptr<pros::IMU>> &imu, bool update = true);
 float calcDeltaTheta(pros::Rotation &tracker1, pros::Rotation &tracker2);

 public:
 Odometry(pros::Rotation &vertical, pros::Rotation &horizontal, double wheelSize, pros::Imu &imu){
  this->vertical   = &vertical;
  this->horizontal = &vertical;
  this->wheelSize  = wheelSize;
  this->imu        = &imu;
 }
 
 pros::Imu* imu;
 Pose pose = Pose(0, 0, 0);

 void init();
 void calibrate(bool calibrateIMU = true);
 void update();
 
};
extern Odometry odom;